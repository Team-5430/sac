package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.firecontrol.ProjectileSimulator;

public class shooter extends SubsystemBase {
    // two motors
    private TalonFX L, R;
    private InterpolatingDoubleTreeMap lut;

    // populate LUT from physics sim using robot measurements
    private void addPoints() {
        var simParams = new ProjectileSimulator.SimParameters(
                0.215, 0.1501, 0.47, 0.2, 1.225,
                1.1346, 0.1016, 1.83, 0.6, 55.0,
                0.001, 2000, 6000, 25, 5.0);

        var sim = new ProjectileSimulator(simParams);
        var genLut = sim.generateLUT();

        for (var entry : genLut.entries()) {
            if (entry.reachable()) {
                lut.put(entry.distanceM(), entry.rpm());
            }
        }
    }

    public shooter(int Left_Motor, int Right_Motor, boolean isInverted) {
        L = new TalonFX(Left_Motor);
        R = new TalonFX(Right_Motor);
        lut = new InterpolatingDoubleTreeMap();
        addPoints();
        motorConfig(isInverted);
    }

    private void motorConfig(boolean invert) {
        var config = new TalonFXConfiguration();

        // tuneables
        var slot0 = config.Slot0;

        slot0.kP = Constants.KPofSHOOTER;

        var feedback = config.Feedback;

        feedback.RotorToSensorRatio = 36 / 30;

        var output = config.MotorOutput;

        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        output.Inverted = invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        L.getConfigurator().apply(config);
        R.getConfigurator().apply(config);

    }

    private VelocityVoltage setter = new VelocityVoltage(0);

    // set rpm to both motors
    private void setRPM(double rpm) {
        setter.withVelocity(rpm);
        L.setControl(setter);
        R.setControl(setter);
    }

    private DutyCycleOut percentageSet = new DutyCycleOut(0);

    private void setPercent(double percentage) {

        percentageSet.withOutput(percentage);

        L.setControl(percentageSet);
        R.setControl(percentageSet);

    }

    public Command SETRPM(double RPM) {
        return Commands.runOnce(() -> setRPM(RPM), this);
    }

    public Command SET_PERCENTAGE(double percentage) {
        return Commands.runOnce(() -> setPercent(percentage), this);
    }

    // supply with distance from vision in order to get correct RPM
    public Command SHOOT(DoubleSupplier distance) {
        return Commands.runOnce(() -> setRPM(lut.get(distance.getAsDouble())), this);
    }

    // direct RPM access for SOTM command (bypasses command scheduler)
    public void setRPMDirect(double rpm) {
        setRPM(rpm);
    }

    public Command STOP() {
        return Commands.runOnce(() -> setPercent(0), this);
    }

}
