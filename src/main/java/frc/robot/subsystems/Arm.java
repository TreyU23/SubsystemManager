package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ArmConstants.kCanID);
    private final TalonFXConfigurator m_cfg = m_motor.getConfigurator();

    private final CANcoder m_canCoder = new CANcoder(ArmConstants.kCanCoderID);

    private final MotionMagicVoltage m_req = new MotionMagicVoltage(0.0);

    private static double m_setpoint = 0.0;

    private final boolean m_clear = true;
    private final double tol = 0.5;

    public Arm() {
        motorConfigs();
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public boolean isClear() {
        if (Math.abs(getPosition() - ArmConstants.kElevator) >= tol) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isFinished() {
        if (Math.abs(getPosition() - m_setpoint) <= tol) {
            return true;
        } else {
            return false;
        }
    }

    public void setPosition(double pose) {
        m_setpoint = pose;
        m_motor.setControl(m_req.withPosition(m_setpoint));
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command setPositionCmd(double pose) {
        return runEnd(()-> setPosition(pose), ()-> setPosition(5.0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Arm Is Clear", m_clear);
        SmartDashboard.putNumber("Arm Setpoint", m_setpoint);
        SmartDashboard.putNumber("Arm Position", getPosition());
        SmartDashboard.putNumber("Arm Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Arm Supply Current", getSupplyCurrent());
    }

    private void motorConfigs() {
        m_cfg.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ArmConstants.kStator)
                .withSupplyCurrentLimit(ArmConstants.kSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        m_cfg.apply(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ArmConstants.kMMVelocity)
                .withMotionMagicAcceleration(ArmConstants.kMMAcceleration)
                .withMotionMagicJerk(ArmConstants.kMMJerk));

        m_cfg.apply(new FeedbackConfigs().withRemoteCANcoder(m_canCoder));
    }
}