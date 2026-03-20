// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.net.Socket;

import javax.xml.crypto.Data;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.fuelintake;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.vision;

public class RobotContainer {

        private final CommandXboxController joystick;
        private final fuelintake Intake;
        private final ButtonBoardControls ButtonBoard;
        public final CommandSwerveDrivetrain drivetrain;

        private double MaxSpeed; // kSpeedAt12Volts desired top speed
        private double MaxAngularRate; // 3/4 of a rotation per second max angular velocity

        private final SwerveRequest.FieldCentric drive; // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake;
        private final SwerveRequest.PointWheelsAt point;
        private final SwerveRequest.RobotCentric robot;

        private shooter LeftShooter = new shooter(Constants.LeftSHOOTERouterID, Constants.LeftSHOOTERinnerID, false);
        private shooter RightShooter = new shooter(Constants.RightSHOOTERinnerID, Constants.RightSHOOTERouterID, true);

        public RobotContainer() {


                joystick = new CommandXboxController(Constants.XboxController);
                ButtonBoard = new ButtonBoardControls(Constants.ButtonBoard);
                drivetrain = TunerConstants.createDrivetrain();
                Intake = new fuelintake(Constants.IntakeRoller, Constants.IntakePivot, Constants.INTAKECANID);

                MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

                point = new SwerveRequest.PointWheelsAt();
                brake = new SwerveRequest.SwerveDriveBrake();
                drive = new SwerveRequest.FieldCentric()
                                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
                                                                                                           // deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                robot = new SwerveRequest.RobotCentric()
                                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
                                                                                                           // deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                configureBindings();
        }

        private void configureBindings() {

                joystick.leftBumper().onTrue(Intake.INTAKE());
                joystick.leftBumper().onFalse(Intake.IDLE());

                //test shooter bindings
               // joystick.rightTrigger().onTrue(LeftShooter.SETRPM(100)
                 //               .alongWith(RightShooter.SETRPM(100)));

/*                 joystick
                                .rightBumper()
                                .onTrue(
                                                LeftShooter.SETRPM(50)
                                                                .alongWith(RightShooter.SETRPM(50)))
                                                                .onFalse(LeftShooter.SETRPM(0).alongWith(RightShooter.SETRPM(0)));;
   */                                                             
                
                                                                

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> robot.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                // Reset our field centric heading to match the robot
                                // facing away from our alliance station wall (0 deg).
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                // Then slowly drive forward (away from us) for 5 seconds.
                                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                                .withVelocityY(0)
                                                .withRotationalRate(0))
                                                .withTimeout(5.0),
                                // Finally idle for the rest of auton
                                drivetrain.applyRequest(() -> idle));

        }


}
