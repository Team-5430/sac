package frc.robot.firecontrol;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// SOTM fire control solver. Accounts for robot velocity, launcher offset,
// latency, and drag to find the correct RPM and heading while driving.
public class ShotCalculator {

  // result of calculate()
  public record LaunchParameters(
      double rpm,
      double timeOfFlightSec,
      Rotation2d driveAngle,
      double driveAngularVelocityRadPerSec,
      boolean isValid,
      double confidence,
      double solvedDistanceM,
      int iterationsUsed,
      boolean warmStartUsed) {

    public static final LaunchParameters INVALID =
        new LaunchParameters(0, 0, new Rotation2d(), 0, false, 0, 0, 0, false);
  }

  // all state the solver needs each cycle
  public record ShotInputs(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      ChassisSpeeds robotVelocity,
      Translation2d hubCenter,
      Translation2d hubForward,
      double visionConfidence,
      double pitchDeg,
      double rollDeg) {

    // convenience for callers without pitch/roll
    public ShotInputs(
        Pose2d robotPose,
        ChassisSpeeds fieldVelocity,
        ChassisSpeeds robotVelocity,
        Translation2d hubCenter,
        Translation2d hubForward,
        double visionConfidence) {
      this(robotPose, fieldVelocity, robotVelocity, hubCenter, hubForward, visionConfidence, 0.0, 0.0);
    }
  }

  // tuning parameters — set to match your robot
  public static class Config {
    public double launcherOffsetX = 0.20; // meters forward of robot center
    public double launcherOffsetY = 0.0;  // meters left of robot center

    public double minScoringDistance = 0.5;
    public double maxScoringDistance = 5.0;

    // newton solver tuning
    public int maxIterations = 25;
    public double convergenceTolerance = 0.001;
    public double tofMin = 0.05;
    public double tofMax = 5.0;

    // below this speed, skip SOTM compensation
    public double minSOTMSpeed = 0.1;
    // above this speed, don't shoot (out of calibration)
    public double maxSOTMSpeed = 3.0;

    // latency compensation
    public double phaseDelayMs = 30.0;
    public double mechLatencyMs = 20.0;

    // horizontal drag damping constant (1/s)
    public double sotmDragCoeff = 0.24;

    // confidence scoring weights
    public double wConvergence = 1.0;
    public double wVelocityStability = 0.8;
    public double wVisionConfidence = 1.2;
    public double wHeadingAccuracy = 1.5;
    public double wDistanceInRange = 0.5;
    public double headingMaxErrorRad = Math.toRadians(15);
    public double headingSpeedScalar = 1.0;
    public double headingReferenceDistance = 2.5;

    // tilt gate
    public double maxTiltDeg = 5.0;

    // rotation between launcher face and robot front (0 = forward)
    public double shooterAngleOffsetRad = 0.0;
  }

  private final Config config;

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionTofMap = new InterpolatingDoubleTreeMap();

  private ShotLUT shotLUT = null;
  private double rpmOffset = 0;

  // solver state (reused across cycles)
  private double previousTOF = -1;
  private double previousSpeed = 0;
  private double prevRobotVx = 0;
  private double prevRobotVy = 0;
  private double prevRobotOmega = 0;

  public ShotCalculator(Config config) { this.config = config; }
  public ShotCalculator() { this(new Config()); }

  // add a distance/RPM/TOF point to the lookup table
  public void loadLUTEntry(double distanceM, double rpm, double tof) {
    rpmMap.put(distanceM, rpm);
    tofMap.put(distanceM, tof);
  }

  double effectiveRPM(double distance) {
    double base = shotLUT != null ? shotLUT.getRPM(distance) : rpmMap.get(distance);
    Double correction = correctionRpmMap.get(distance);
    return base + (correction != null ? correction : 0.0) + rpmOffset;
  }

  double effectiveTOF(double distance) {
    double base = shotLUT != null ? shotLUT.getTOF(distance) : tofMap.get(distance);
    Double correction = correctionTofMap.get(distance);
    return base + (correction != null ? correction : 0.0);
  }

  // drag-adjusted effective TOF
  private double dragCompensatedTOF(double tof) {
    double c = config.sotmDragCoeff;
    if (c < 1e-6) return tof;
    return (1.0 - Math.exp(-c * tof)) / c;
  }

  private static final double DERIV_H = 0.01;

  double tofMapDerivative(double d) {
    return (effectiveTOF(d + DERIV_H) - effectiveTOF(d - DERIV_H)) / (2.0 * DERIV_H);
  }

  // main solver — call once per cycle
  public LaunchParameters calculate(ShotInputs inputs) {
    if (inputs == null || inputs.robotPose() == null
        || inputs.fieldVelocity() == null || inputs.robotVelocity() == null) {
      return LaunchParameters.INVALID;
    }

    Pose2d rawPose = inputs.robotPose();
    ChassisSpeeds fieldVel = inputs.fieldVelocity();
    ChassisSpeeds robotVel = inputs.robotVelocity();

    double poseX = rawPose.getX();
    double poseY = rawPose.getY();
    if (Double.isNaN(poseX) || Double.isNaN(poseY)
        || Double.isInfinite(poseX) || Double.isInfinite(poseY)) {
      return LaunchParameters.INVALID;
    }

    // second-order pose prediction (v*dt + 0.5*a*dt^2)
    double dt = config.phaseDelayMs / 1000.0;
    double ax = (robotVel.vxMetersPerSecond - prevRobotVx) / 0.02;
    double ay = (robotVel.vyMetersPerSecond - prevRobotVy) / 0.02;
    double aOmega = (robotVel.omegaRadiansPerSecond - prevRobotOmega) / 0.02;
    Pose2d compensatedPose = rawPose.exp(new Twist2d(
        robotVel.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
        robotVel.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
        robotVel.omegaRadiansPerSecond * dt + 0.5 * aOmega * dt * dt));
    prevRobotVx = robotVel.vxMetersPerSecond;
    prevRobotVy = robotVel.vyMetersPerSecond;
    prevRobotOmega = robotVel.omegaRadiansPerSecond;

    double robotX = compensatedPose.getX();
    double robotY = compensatedPose.getY();
    double heading = compensatedPose.getRotation().getRadians();

    Translation2d hubCenter = inputs.hubCenter();
    double hubX = hubCenter.getX();
    double hubY = hubCenter.getY();

    // behind-hub check
    Translation2d hubForward = inputs.hubForward();
    double dot = (hubX - robotX) * hubForward.getX() + (hubY - robotY) * hubForward.getY();
    if (dot < 0) return LaunchParameters.INVALID;

    // tilt gate
    if (Math.abs(inputs.pitchDeg()) > config.maxTiltDeg
        || Math.abs(inputs.rollDeg()) > config.maxTiltDeg) {
      return LaunchParameters.INVALID;
    }

    // transform to launcher position
    double cosH = Math.cos(heading);
    double sinH = Math.sin(heading);
    double launcherX = robotX + config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherY = robotY + config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;

    // launcher velocity includes rotational component
    double launcherFieldOffX = config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherFieldOffY = config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;
    double omega = fieldVel.omegaRadiansPerSecond;
    double vx = fieldVel.vxMetersPerSecond + (-launcherFieldOffY) * omega;
    double vy = fieldVel.vyMetersPerSecond + launcherFieldOffX * omega;

    double rx = hubX - launcherX;
    double ry = hubY - launcherY;
    double distance = Math.hypot(rx, ry);

    if (distance < config.minScoringDistance || distance > config.maxScoringDistance) {
      return LaunchParameters.INVALID;
    }

    double robotSpeed = Math.hypot(vx, vy);
    if (robotSpeed > config.maxSOTMSpeed) return LaunchParameters.INVALID;

    boolean velocityFiltered = robotSpeed < config.minSOTMSpeed;
    double solvedTOF;
    double projDist;
    int iterationsUsed;
    boolean warmStartUsed;

    if (velocityFiltered) {
      // static shot
      solvedTOF = effectiveTOF(distance);
      projDist = distance;
      iterationsUsed = 0;
      warmStartUsed = false;
    } else {
      // newton-method SOTM solver
      int maxIter = config.maxIterations;
      double convTol = config.convergenceTolerance;

      double tof;
      if (previousTOF > 0) {
        tof = previousTOF;
        warmStartUsed = true;
      } else {
        tof = effectiveTOF(distance);
        warmStartUsed = false;
      }

      projDist = distance;
      iterationsUsed = 0;

      for (int i = 0; i < maxIter; i++) {
        double prevTOF = tof;
        double c = config.sotmDragCoeff;
        double dragExp = c < 1e-6 ? 1.0 : Math.exp(-c * tof);
        double driftTOF = c < 1e-6 ? tof : (1.0 - dragExp) / c;

        double prx = rx - vx * driftTOF;
        double pry = ry - vy * driftTOF;
        projDist = Math.hypot(prx, pry);

        if (projDist < 0.01) {
          tof = effectiveTOF(distance);
          iterationsUsed = maxIter + 1;
          break;
        }

        double lookupTOF = effectiveTOF(projDist);
        double dPrime = -dragExp * (prx * vx + pry * vy) / projDist;
        double gPrime = tofMapDerivative(projDist);
        double f = lookupTOF - tof;
        double fPrime = gPrime * dPrime - 1.0;

        if (Math.abs(fPrime) > 0.01) {
          tof = tof - f / fPrime;
        } else {
          tof = lookupTOF;
        }

        tof = MathUtil.clamp(tof, config.tofMin, config.tofMax);
        iterationsUsed = i + 1;
        if (Math.abs(tof - prevTOF) < convTol) break;
      }

      if (tof > config.tofMax || tof < 0.0 || Double.isNaN(tof)) {
        tof = effectiveTOF(distance);
        iterationsUsed = maxIter + 1;
      }

      solvedTOF = tof;
    }

    previousTOF = solvedTOF;
    double effectiveTOF = solvedTOF + config.mechLatencyMs / 1000.0;
    double effectiveRPMValue = effectiveRPM(projDist);

    // drive angle: aim at velocity-compensated target
    double compTargetX, compTargetY;
    if (velocityFiltered) {
      compTargetX = hubX;
      compTargetY = hubY;
    } else {
      double headingDriftTOF = dragCompensatedTOF(solvedTOF);
      compTargetX = hubX - vx * headingDriftTOF;
      compTargetY = hubY - vy * headingDriftTOF;
    }
    double aimX = compTargetX - robotX;
    double aimY = compTargetY - robotY;
    Rotation2d driveAngle = new Rotation2d(aimX, aimY);
    if (config.shooterAngleOffsetRad != 0.0) {
      driveAngle = driveAngle.plus(new Rotation2d(config.shooterAngleOffsetRad));
    }

    double headingErrorRad = MathUtil.angleModulus(driveAngle.getRadians() - heading);

    // angular velocity feedforward
    double driveAngularVelocity = 0;
    if (!velocityFiltered && distance > 0.1) {
      double tangentialVel = (ry * vx - rx * vy) / distance;
      driveAngularVelocity = tangentialVel / distance;
    }

    // solver convergence quality
    double solverQuality;
    if (velocityFiltered) {
      solverQuality = 1.0;
    } else {
      int maxIter = config.maxIterations;
      if (iterationsUsed > maxIter) {
        solverQuality = 0.0;
      } else if (iterationsUsed <= 3) {
        solverQuality = 1.0;
      } else {
        solverQuality = MathUtil.interpolate(1.0, 0.1,
            (double) (iterationsUsed - 3) / (maxIter - 3));
      }
    }

    double confidence = computeConfidence(
        solverQuality, robotSpeed, headingErrorRad, distance, inputs.visionConfidence());
    previousSpeed = robotSpeed;

    return new LaunchParameters(
        effectiveRPMValue, effectiveTOF, driveAngle, driveAngularVelocity,
        true, confidence, distance, iterationsUsed, warmStartUsed);
  }

  // 5-component weighted geometric mean confidence
  private double computeConfidence(
      double solverQuality, double currentSpeed, double headingErrorRad,
      double distance, double visionConfidence) {

    double convergenceQuality = solverQuality;
    double speedDelta = Math.abs(currentSpeed - previousSpeed);
    double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0, 1);
    double visionConf = MathUtil.clamp(visionConfidence, 0, 1);

    double distanceScale = MathUtil.clamp(config.headingReferenceDistance / distance, 0.5, 2.0);
    double speedScale = 1.0 / (1.0 + config.headingSpeedScalar * currentSpeed);
    double scaledMaxError = config.headingMaxErrorRad * distanceScale * speedScale;
    double headingAccuracy = MathUtil.clamp(1.0 - Math.abs(headingErrorRad) / scaledMaxError, 0, 1);

    double rangeSpan = config.maxScoringDistance - config.minScoringDistance;
    double rangeFraction = (distance - config.minScoringDistance) / rangeSpan;
    double distInRange = MathUtil.clamp(1.0 - 2.0 * Math.abs(rangeFraction - 0.5), 0, 1);

    double[] c = {convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange};
    double[] w = {config.wConvergence, config.wVelocityStability,
        config.wVisionConfidence, config.wHeadingAccuracy, config.wDistanceInRange};

    double sumW = 0, logSum = 0;
    for (int i = 0; i < 5; i++) {
      if (c[i] <= 0) return 0;
      logSum += w[i] * Math.log(c[i]);
      sumW += w[i];
    }
    if (sumW <= 0) return 0;
    return MathUtil.clamp(Math.exp(logSum / sumW) * 100.0, 0, 100);
  }

  public void addRpmCorrection(double distance, double deltaRpm) {
    correctionRpmMap.put(distance, deltaRpm);
  }

  public void addTofCorrection(double distance, double deltaTof) {
    correctionTofMap.put(distance, deltaTof);
  }

  public void clearCorrections() {
    correctionRpmMap.clear();
    correctionTofMap.clear();
  }

  // copilot RPM trim, clamped +/- 200
  public void adjustOffset(double delta) {
    rpmOffset = MathUtil.clamp(rpmOffset + delta, -200, 200);
  }

  public void resetOffset() { rpmOffset = 0; }
  public double getOffset() { return rpmOffset; }

  public double getTimeOfFlight(double distanceM) { return effectiveTOF(distanceM); }

  public double getBaseRPM(double distance) {
    if (shotLUT != null) return shotLUT.getRPM(distance);
    return rpmMap.get(distance);
  }

  // reset warm start after pose reset
  public void resetWarmStart() {
    previousTOF = -1;
    previousSpeed = 0;
    prevRobotVx = 0;
    prevRobotVy = 0;
    prevRobotOmega = 0;
  }

  public void loadShotLUT(ShotLUT lut) { this.shotLUT = lut; }

  public double getHoodAngle(double distance) {
    if (shotLUT != null) return shotLUT.getAngle(distance);
    return 0;
  }
}
