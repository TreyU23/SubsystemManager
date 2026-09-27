package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SmallPeiceConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator.SmallPeice;

public class RobotContainer {
  private final SmallPeice m_smallpPeice = new SmallPeice();
  private final Arm m_arm = new Arm();
  private final Elevator m_elevator = new Elevator();

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandGenericHID m_operatorBoard = new CommandGenericHID(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightTrigger()
        .whileTrue(m_smallpPeice.scoreCmd(SmallPeiceConstants.kScoringVoltage));

    // HID buttons start at 1. Confirm these numbers against the operator board mapping.
    m_driverController.leftTrigger().and(m_operatorBoard.button(1))
        .whileTrue(moveToLevel(ElevatorConstants.kLevelOne, ArmConstants.kLevelOne));
    m_driverController.leftTrigger().and(m_operatorBoard.button(2))
        .whileTrue(moveToLevel(ElevatorConstants.kLevelTwo, ArmConstants.kLevelTwo));
    m_driverController.leftTrigger().and(m_operatorBoard.button(3))
        .whileTrue(moveToLevel(ElevatorConstants.kLevelThree, ArmConstants.kLevelThree));

    m_driverController.leftBumper().whileTrue(new ConditionalCommand(
        homeAndIntake(),
        m_arm.setPositionCmd(ArmConstants.kClear)
            .until(m_arm::isClear)
            .andThen(homeAndIntake()),
        () -> m_arm.isClear() || m_arm.isIntakeSideClear()));
  }

  private Command moveToLevel(double elevatorLevel, double armLevel) {
    Command move = new ConditionalCommand(
        m_arm.setPositionCmd(armLevel)
            .alongWith(m_elevator.setPositionCmd(elevatorLevel)),
        m_arm.setPositionCmd(ArmConstants.kClear)
            .until(m_arm::isClear)
            .andThen(m_arm.setPositionCmd(armLevel)
                .alongWith(m_elevator.setPositionCmd(elevatorLevel))),
        m_arm::isClear);

    return move.alongWith(Commands.run(() -> m_smallpPeice.setCanShoot(
        m_arm.isAtPosition(armLevel) && m_elevator.isAtPosition(elevatorLevel))))
        .finallyDo(() -> m_smallpPeice.setCanShoot(false));
  }

  private Command homeAndIntake() {
    return m_elevator.setPositionCmd(ElevatorConstants.kHome)
        .until(() -> m_elevator.isAtPosition(ElevatorConstants.kHome))
        .andThen(m_arm.setPositionCmd(ArmConstants.kIntake)
            .until(() -> m_arm.isAtPosition(ArmConstants.kIntake)))
        .andThen(m_smallpPeice.setVoltageCmd(SmallPeiceConstants.kCoastVoltage));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
  }
}
