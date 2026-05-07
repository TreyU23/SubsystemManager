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
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ElevatorConstants.kCanID);
    private final TalonFXConfigurator m_cfg = m_motor.getConfigurator();

    private final CANcoder m_canCoder = new CANcoder(ElevatorConstants.kCanCoderID);

    private final MotionMagicVoltage m_req = new MotionMagicVoltage(0.0);

    private static double m_setpoint = 0.0;
    private static double m_lastSetpoint = 0.0;

    private final double tol = 0.5;

    public Elevator() {
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

    public void setPosition(double pose) {
        m_setpoint = pose;
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command setPositionCmd(double pose) {
        return runEnd(()-> setPosition(pose), ()-> setPosition(5.0));
    }

    public boolean goingDown() {
        if (m_setpoint < m_lastSetpoint) {
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

    @Override
    public void periodic() {
        m_motor.setControl(m_req.withPosition(m_setpoint));

        SmartDashboard.putNumber("Elevator Setpoint", m_setpoint);
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Elevator Supply Current", getSupplyCurrent());

        m_lastSetpoint = m_setpoint;
    }

    private void motorConfigs() {
        m_cfg.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ElevatorConstants.kStator)
                .withSupplyCurrentLimit(ElevatorConstants.kSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        m_cfg.apply(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ElevatorConstants.kMMVelocity)
                .withMotionMagicAcceleration(ElevatorConstants.kMMAcceleration)
                .withMotionMagicJerk(ElevatorConstants.kMMJerk));

        m_cfg.apply(new FeedbackConfigs().withRemoteCANcoder(m_canCoder));
    }
}

