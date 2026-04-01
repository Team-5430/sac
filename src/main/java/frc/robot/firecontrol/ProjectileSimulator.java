package frc.robot.firecontrol;

import java.util.ArrayList;
import java.util.List;

// RK4 projectile sim with drag and Magnus lift. Binary searches RPM at each
// distance to hit the target height, then generates a full shooter LUT.
public class ProjectileSimulator {

  public record SimParameters(
      double ballMassKg,
      double ballDiameterM,
      double dragCoeff,
      double magnusCoeff,
      double airDensity,
      double exitHeightM,
      double wheelDiameterM,
      double targetHeightM,
      double slipFactor,
      double fixedLaunchAngleDeg,
      double dt,
      double rpmMin,
      double rpmMax,
      int binarySearchIters,
      double maxSimTime) {}

  public record TrajectoryResult(
      double zAtTarget, double tof, boolean reachedTarget, double maxHeight, double apexX) {}

  public record LUTEntry(double distanceM, double rpm, double tof, boolean reachable) {}

  public record GeneratedLUT(
      List<LUTEntry> entries,
      SimParameters params,
      int reachableCount,
      int unreachableCount,
      double maxRangeM,
      long generationTimeMs) {}

  private final SimParameters params;
  private final double kDrag;
  private final double kMagnus;
  private double magnusSign = 1.0;

  public ProjectileSimulator(SimParameters params) {
    this.params = params;
    double area = Math.PI * (params.ballDiameterM() / 2.0) * (params.ballDiameterM() / 2.0);
    this.kDrag = (params.airDensity() * params.dragCoeff() * area) / (2.0 * params.ballMassKg());
    this.kMagnus = (params.airDensity() * params.magnusCoeff() * area) / (2.0 * params.ballMassKg());
  }

  public ProjectileSimulator(SimParameters params, double magnusSign) {
    this(params);
    this.magnusSign = magnusSign;
  }

  // RPM to ball exit speed accounting for slip
  public double exitVelocity(double rpm) {
    return params.slipFactor() * rpm * Math.PI * params.wheelDiameterM() / 60.0;
  }

  public TrajectoryResult simulate(double rpm, double targetDistanceM) {
    return simulate(rpm, targetDistanceM, params.fixedLaunchAngleDeg());
  }

  public TrajectoryResult simulate(double rpm, double targetDistanceM, double launchAngleDeg) {
    double v0 = exitVelocity(rpm);
    double launchRad = Math.toRadians(launchAngleDeg);
    double vx = v0 * Math.cos(launchRad);
    double vz = v0 * Math.sin(launchRad);

    double x = 0;
    double z = params.exitHeightM();
    double dt = params.dt();
    double maxHeight = z;
    double apexX = 0;
    double t = 0;

    while (t < params.maxSimTime()) {
      double prevX = x;
      double prevZ = z;

      // RK4 integration step
      double[] state = {x, z, vx, vz};
      double[] k1 = derivatives(state);
      double[] s2 = addScaled(state, k1, dt / 2.0);
      double[] k2 = derivatives(s2);
      double[] s3 = addScaled(state, k2, dt / 2.0);
      double[] k3 = derivatives(s3);
      double[] s4 = addScaled(state, k3, dt);
      double[] k4 = derivatives(s4);

      x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
      z += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
      vx += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
      vz += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
      t += dt;

      if (z > maxHeight) { maxHeight = z; apexX = x; }

      if (x >= targetDistanceM) {
        double frac = (targetDistanceM - prevX) / (x - prevX);
        double zAtTarget = prevZ + frac * (z - prevZ);
        double tofAtTarget = t - dt + frac * dt;
        return new TrajectoryResult(zAtTarget, tofAtTarget, true, maxHeight, apexX);
      }

      if (z < 0) return new TrajectoryResult(0, t, false, maxHeight, apexX);
    }

    return new TrajectoryResult(0, params.maxSimTime(), false, maxHeight, apexX);
  }

  // state = [x, z, vx, vz]
  private double[] derivatives(double[] state) {
    double svx = state[2];
    double svz = state[3];
    double speed = Math.hypot(svx, svz);
    double ax = -kDrag * speed * svx;
    double az = -9.81 - kDrag * speed * svz + magnusSign * kMagnus * speed * speed;
    return new double[] {svx, svz, ax, az};
  }

  private static double[] addScaled(double[] base, double[] delta, double scale) {
    return new double[] {
      base[0] + delta[0] * scale, base[1] + delta[1] * scale,
      base[2] + delta[2] * scale, base[3] + delta[3] * scale
    };
  }

  public LUTEntry findRPMForDistance(double distanceM) {
    return findRPMForDistance(distanceM, params.fixedLaunchAngleDeg());
  }

  // binary search for RPM that lands the ball at target height
  public LUTEntry findRPMForDistance(double distanceM, double launchAngleDeg) {
    double heightTolerance = 0.02;
    double lo = params.rpmMin();
    double hi = params.rpmMax();

    TrajectoryResult maxCheck = simulate(hi, distanceM, launchAngleDeg);
    if (!maxCheck.reachedTarget()) return new LUTEntry(distanceM, 0, 0, false);

    double bestRpm = hi;
    double bestTof = maxCheck.tof();
    double bestError = Math.abs(maxCheck.zAtTarget() - params.targetHeightM());

    for (int i = 0; i < params.binarySearchIters(); i++) {
      double mid = (lo + hi) / 2.0;
      TrajectoryResult result = simulate(mid, distanceM, launchAngleDeg);

      if (!result.reachedTarget()) { lo = mid; continue; }

      double error = result.zAtTarget() - params.targetHeightM();
      double absError = Math.abs(error);

      if (absError < bestError) { bestRpm = mid; bestTof = result.tof(); bestError = absError; }
      if (absError < heightTolerance) return new LUTEntry(distanceM, mid, result.tof(), true);

      if (error > 0) hi = mid; else lo = mid;
    }

    return new LUTEntry(distanceM, bestRpm, bestTof, bestError < 0.10);
  }

  // generate full LUT: 0.50m to 5.00m in 5cm steps
  public GeneratedLUT generateLUT() { return generateLUT(0.50, 5.00, 0.05); }

  public GeneratedLUT generateLUT(double minDistM, double maxDistM, double stepM) {
    long startMs = System.currentTimeMillis();
    List<LUTEntry> entries = new ArrayList<>();
    int reachable = 0, unreachable = 0;
    double maxRange = 0;

    for (double distance = minDistM; distance <= maxDistM + stepM * 0.01; distance += stepM) {
      distance = Math.round(distance * 100.0) / 100.0;
      LUTEntry entry = findRPMForDistance(distance);
      entries.add(entry);
      if (entry.reachable()) { reachable++; maxRange = distance; } else { unreachable++; }
    }

    return new GeneratedLUT(entries, params, reachable, unreachable, maxRange,
        System.currentTimeMillis() - startMs);
  }

  // generate LUT as ShotLUT with fixed angle baked in
  public ShotLUT generateShotLUT() {
    GeneratedLUT gen = generateLUT();
    ShotLUT lut = new ShotLUT();
    for (LUTEntry entry : gen.entries()) {
      if (entry.reachable()) {
        lut.put(entry.distanceM(),
            new ShotParameters(entry.rpm(), params.fixedLaunchAngleDeg(), entry.tof()));
      }
    }
    return lut;
  }
}
