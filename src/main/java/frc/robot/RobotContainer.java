// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoLock;
import frc.robot.firecontrol.ProjectileSimulator;
import frc.robot.firecontrol.ShotCalculator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.fuelintake;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.vision;

public class RobotContainer {

        // CANGE TO FEILD WHEN AT COMP

        private final CommandXboxController joystick;
        private final fuelintake Intake;
        private final ButtonBoardControls ButtonBoard;
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final vision Vision;
        private final Indexer Indexer;

        private shooter LeftShooter = new shooter(Constants.LeftSHOOTERouterID, Constants.LeftSHOOTERinnerID, false);
        private shooter RightShooter = new shooter(Constants.RightSHOOTERinnerID, Constants.RightSHOOTERouterID, true);

        // SOTM fire control solver
        private final ShotCalculator shotCalculator;

        private double MaxSpeed; // kSpeedAt12Volts desired top speed
        private double MaxAngularRate; // 3/4 of a rotation per second max angular velocity


        //drive commands
        private final SwerveRequest.FieldCentric field; // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake;
        private final SwerveRequest.PointWheelsAt point;

        // SOTM auto-aim command
        private final AutoLock autoLock;

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);

                joystick = new CommandXboxController(Constants.XboxController);
                ButtonBoard = new ButtonBoardControls(Constants.ButtonBoard);




                Intake = new fuelintake(Constants.IntakeRoller, Constants.IntakePivot, Constants.INTAKECANID);
                Indexer = new Indexer(Constants.IndexerID);
                Vision = new vision();

                MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

                point = new SwerveRequest.PointWheelsAt();
                brake = new SwerveRequest.SwerveDriveBrake();
                field = new SwerveRequest.FieldCentric()
                                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
                                                                                                           // deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                // configure SOTM solver
                shotCalculator = configureShotCalculator();

                autoLock = new AutoLock(
                        drivetrain,
                        LeftShooter,
                        RightShooter,
                        Vision,
                        shotCalculator,
                        () -> -joystick.getLeftY() * MaxSpeed,
                        () -> -joystick.getLeftX() * MaxSpeed);

                configureBindings();



        }

        // SOTM solver config — tune these to match your robot
        private ShotCalculator configureShotCalculator() {
                ShotCalculator.Config config = new ShotCalculator.Config();
                config.launcherOffsetX = 0.20;  // meters forward of center, measure from CAD
                config.launcherOffsetY = 0.0;
                config.minScoringDistance = 0.5;
                config.maxScoringDistance = 5.0;
                config.phaseDelayMs = 30.0;     // vision pipeline latency
                config.mechLatencyMs = 50.0;    // flywheel spinup lag
                config.sotmDragCoeff = 0.24;    // horizontal drag damping

                ShotCalculator calc = new ShotCalculator(config);

                // generate LUT from physics sim using robot measurements
                var simParams = new ProjectileSimulator.SimParameters(
                        0.215,    // ball mass kg (game manual)
                        0.1501,   // ball diameter m (game manual)
                        0.47,     // drag coeff (smooth sphere)
                        0.2,      // Magnus coeff
                        1.225,    // air density kg/m^3
                        1.1346,   // exit height from floor (CAD)
                        0.1016,   // flywheel wheel diameter (calipers)
                        1.83,     // target height (game manual)
                        0.6,      // slip factor (tune on robot) -> if overshoot lower it fr
                        55.0,     // launch angle degrees
                        0.001,    // sim timestep
                        2000, 6000, 25, 5.0); // RPM range, search iters, max sim time

                var sim = new ProjectileSimulator(simParams);
                var lut = sim.generateLUT();

                // load every reachable point into the SOTM solver
                for (var entry : lut.entries()) {
                        if (entry.reachable()) {
                                calc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
                        }
                }

                return calc;
        }

        private void configureBindings() {
                // change all mechanismz to gunner at comp, bring button boeard

                // SOTM auto-aim toggle on right bumper
                joystick.rightBumper().toggleOnTrue(autoLock);

                // shoot + index on right trigger
                joystick.rightTrigger().onTrue(LeftShooter.SHOOT(() -> Vision.getDistance())
                                .alongWith(RightShooter.SHOOT(() -> Vision.getDistance()))
                                .alongWith(Indexer.INDEX_IN()));

                // intake on left bumper
                joystick.leftBumper().onTrue(Intake.INTAKE());
                joystick.leftBumper().onFalse(Intake.IDLE());

                ButtonBoard.Button7().onTrue(Intake.INTAKE()).onFalse(Intake.STOP());
                ButtonBoard.Button8().onTrue(Intake.OUTTAKE()).onFalse(Intake.STOP());
                ButtonBoard.Button12().onTrue(Intake.IDLE());
                ButtonBoard.Button11().toggleOnFalse(Indexer.INDEX_IDLE()).toggleOnTrue(Indexer.INDEXCYCLE());

                ButtonBoard.Button10().onTrue(LeftShooter.SET_PERCENTAGE(0.6)
                                .alongWith(RightShooter.SET_PERCENTAGE(0.6)));

                ButtonBoard.Button10().onFalse(LeftShooter.SET_PERCENTAGE(0)
                                .alongWith(RightShooter.SET_PERCENTAGE(0)));

                ButtonBoard.Button9().onTrue(LeftShooter.SET_PERCENTAGE(0.7)
                                .alongWith(RightShooter.SET_PERCENTAGE(0.7)));

                joystick.b().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> field.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                               // negative X (left)
                                                .withRotationalRate(joystick.getRightX() * MaxAngularRate)) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)

                );

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        }

        public Command autoCommand(){
                return autoChooser.getSelected();
        }

}
