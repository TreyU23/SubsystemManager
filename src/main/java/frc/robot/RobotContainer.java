package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    m_driverController.leftTrigger().and(m_operatorBoard.button(1))
        .whileTrue(moveToLevel(ElevatorConstants.kLevelOne, ArmConstants.kLevelOne));
    m_driverController.leftTrigger().and(m_operatorBoard.button(2))
        .whileTrue(moveToLevel(ElevatorConstants.kLevelTwo, ArmConstants.kLevelTwo));
    m_driverController.leftTrigger().and(m_operatorBoard.button(3))
        .whileTrue(moveToLevel(ElevatorConstants.kLevelThree, ArmConstants.kLevelThree));

    m_driverController.leftBumper().whileTrue(homeAndIntake());
  }

  private Command clearArm() {
    return m_arm.setPositionCmd(ArmConstants.kClear).until(m_arm::isClear);
  }

  private Command moveToLevel(double elevatorLevel, double armLevel) {
    return Commands.sequence(
            Commands.runOnce(() -> m_smallpPeice.setCanShoot(false)),
            clearArm(),
            m_elevator.setPositionCmd(elevatorLevel)
                .alongWith(m_arm.setPositionCmd(ArmConstants.kClear))
                .until(() -> m_elevator.isAtPosition(elevatorLevel)),
            Commands.parallel(
                m_arm.setPositionCmd(armLevel),
                m_elevator.setPositionCmd(elevatorLevel),
                Commands.run(() -> m_smallpPeice.setCanShoot(
                    m_arm.isAtPosition(armLevel) && m_elevator.isAtPosition(elevatorLevel)))))
        .finallyDo(() -> m_smallpPeice.setCanShoot(false));
  }

  private Command homeAndIntake() {
    return Commands.sequence(
        clearArm().unless(() -> m_arm.isClear() || m_arm.isIntakeSideClear()),
        m_elevator.setPositionCmd(ElevatorConstants.kHome)
            .until(() -> m_elevator.isAtPosition(ElevatorConstants.kHome)),
        Commands.parallel(
            m_arm.setPositionCmd(ArmConstants.kIntake),
            m_elevator.setPositionCmd(ElevatorConstants.kHome),
            Commands.waitUntil(() -> m_arm.isAtPosition(ArmConstants.kIntake))
                .andThen(m_smallpPeice.setVoltageCmd(SmallPeiceConstants.kCoastVoltage))));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
