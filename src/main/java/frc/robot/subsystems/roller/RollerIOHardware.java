package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.RollerConstants;

public class RollerIOHardware {
    private final TalonFX m_rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_rollerMotor.getVelocity();
    private final StatusSignal<Current> m_currentSignal = m_rollerMotor.getStatorCurrent();

    // constructor
    public RollerIOHardware() {
        var talonFXConfigurator = m_rollerMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = RollerConstants.MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    // getters
    public double getVelocity() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*RollerConstants.METERS_PER_ROTATION;
    }

    public double getCurrent() {
        return m_currentSignal.refresh().getValue().in(Amp);
    }

    // setters
    public void motorOff() {
        m_rollerMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_rollerMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(0);
    }

    public void setVoltage(double value) {
        SmartDashboard.putNumber("roller/voltage", value);
        m_rollerMotor.setControl(new VoltageOut(value));
    }
}
