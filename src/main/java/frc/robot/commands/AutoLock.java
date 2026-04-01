package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.firecontrol.ShotCalculator;
import frc.robot.firecontrol.ShotCalculator.LaunchParameters;
import frc.robot.firecontrol.ShotCalculator.ShotInputs;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.vision;
import java.util.function.DoubleSupplier;

// SOTM auto-aim: rotates robot toward velocity-compensated target while driving
public class AutoLock extends Command {

    private CommandSwerveDrivetrain drive;
    private shooter LeftShooter, RightShooter;
    private vision Vision;
    private ShotCalculator calculator;

    private SwerveRequest.FieldCentricFacingAngle aim = new SwerveRequest.FieldCentricFacingAngle();

    // hub position and forward vector (toward blue alliance wall)
    private static final Translation2d HUB_POSITION = new Translation2d(4.6, 4);
    private static final Translation2d HUB_FORWARD = new Translation2d(-1, 0);

    private DoubleSupplier xSupplier, ySupplier;

    // minimum confidence to spin up shooter
    private static final double MIN_CONFIDENCE = 30;

    public AutoLock(
            CommandSwerveDrivetrain drive,
            shooter LeftShooter,
            shooter RightShooter,
            vision Vision,
            ShotCalculator calculator,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.drive = drive;
        this.LeftShooter = LeftShooter;
        this.RightShooter = RightShooter;
        this.Vision = Vision;
        this.calculator = calculator;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        addRequirements(drive, LeftShooter, RightShooter);
    }

    @Override
    public void initialize() {
        calculator.resetWarmStart();
    }

    @Override
    public void execute() {
        // build field-relative velocity from robot-relative speeds + heading
        ChassisSpeeds robotSpeeds = drive.getState().Speeds;
        double heading = drive.getState().Pose.getRotation().getRadians();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // rotate robot-frame speeds to field-frame
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
                robotSpeeds.vxMetersPerSecond * cos - robotSpeeds.vyMetersPerSecond * sin,
                robotSpeeds.vxMetersPerSecond * sin + robotSpeeds.vyMetersPerSecond * cos,
                robotSpeeds.omegaRadiansPerSecond);

        // vision confidence: 1.0 if we have a recent pose, 0.0 otherwise
        double visionConf = Vision.getDistance() > 0.01 ? 1.0 : 0.0;

        ShotInputs inputs = new ShotInputs(
                drive.getState().Pose,
                fieldSpeeds,
                robotSpeeds,
                HUB_POSITION,
                HUB_FORWARD,
                visionConf);

        LaunchParameters result = calculator.calculate(inputs);

        // publish telemetry
        SmartDashboard.putBoolean("SOTM/Valid", result.isValid());
        SmartDashboard.putNumber("SOTM/Confidence", result.confidence());
        SmartDashboard.putNumber("SOTM/RPM", result.rpm());
        SmartDashboard.putNumber("SOTM/Distance", result.solvedDistanceM());
        SmartDashboard.putNumber("SOTM/Iterations", result.iterationsUsed());

        if (result.isValid()) {
            // aim at SOTM-compensated heading, keep joystick translation
            drive.setControl(
                aim.withTargetDirection(result.driveAngle())
                   .withVelocityX(xSupplier.getAsDouble())
                   .withVelocityY(ySupplier.getAsDouble()));

            // spin up if confidence is high enough
            if (result.confidence() > MIN_CONFIDENCE) {
                LeftShooter.setRPMDirect(result.rpm());
                RightShooter.setRPMDirect(result.rpm());
            }
        } else {
            // no valid solution, just aim at hub statically
            Translation2d robotPos = drive.getState().Pose.getTranslation();
            Translation2d toHub = HUB_POSITION.minus(robotPos);
            Rotation2d angleToHub = new Rotation2d(toHub.getX(), toHub.getY());

            drive.setControl(
                aim.withTargetDirection(angleToHub)
                   .withVelocityX(xSupplier.getAsDouble())
                   .withVelocityY(ySupplier.getAsDouble()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(new SwerveRequest.Idle());
        // stop shooter wheels
        LeftShooter.setRPMDirect(0);
        RightShooter.setRPMDirect(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
