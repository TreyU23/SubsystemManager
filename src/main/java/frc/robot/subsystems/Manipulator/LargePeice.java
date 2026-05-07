package frc.robot.subsystems.Manipulator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LargePeiceConstants;

public class LargePeice extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(LargePeiceConstants.kCanID);
    private final TalonFXConfigurator m_cfg = m_motor.getConfigurator();

    private double m_setpoint = 0.0;
    private boolean m_canShoot = false;

    public LargePeice() {
        motorConfigs();
    }

    public double getVelocity() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    public double getStatorCurret() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public void setVoltage(double volts) {
        if (m_canShoot) {
            m_setpoint = volts;
        } else {
            m_setpoint = 0.0;
        }
        m_motor.setVoltage(m_setpoint);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command stopCmd() {
        return runOnce(()-> stop());
    }

    public Command setVoltageCmd(double volts) {
        return runEnd(() -> setVoltage(volts), () -> stop());
    }

    public Command setCanShoot(boolean value) {
        return runOnce(()-> {
            m_canShoot = value;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("largePeice Setpoint", m_setpoint);
        SmartDashboard.putNumber("largePeice Velocity", getVelocity());
        SmartDashboard.putNumber("largePeice Stator Current", getStatorCurret());
    }

    private void motorConfigs() {
        m_cfg.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(LargePeiceConstants.kStator)
                .withSupplyCurrentLimit(LargePeiceConstants.kSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));
    }
}
