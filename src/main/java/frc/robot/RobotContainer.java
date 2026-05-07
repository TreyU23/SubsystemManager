package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
        .whileTrue(new ConditionalCommand(m_smallpPeice.setVoltageCmd(SmallPeiceConstants.kScoringVoltage), m_smallpPeice.stopCmd(), m_smallpPeice::canShoot));

    m_driverController.leftTrigger().and(m_operatorBoard.button(0))
        .whileTrue(new ConditionalCommand(
            m_elevator.setPositionCmd(ElevatorConstants.kLevelOne)
                .alongWith(m_arm.setPositionCmd(ArmConstants.kLevelOne)
                    .alongWith(m_smallpPeice.setCanShoot(true))),
            m_arm.setPositionCmd(ArmConstants.kLevelOne)
                .alongWith(new WaitUntilCommand(m_arm::isClear)
                    .andThen(m_elevator.setPositionCmd(ElevatorConstants.kLevelOne)
                      .alongWith(m_smallpPeice.setCanShoot(true)))),
            m_arm::isClear));

    m_driverController.leftTrigger().and(m_operatorBoard.button(1))
        .whileTrue(new ConditionalCommand(
            m_elevator.setPositionCmd(ElevatorConstants.kLevelTwo)
                .alongWith(m_arm.setPositionCmd(ArmConstants.kLevelTwo)
                    .alongWith(m_smallpPeice.setCanShoot(true))),
            m_arm.setPositionCmd(ArmConstants.kLevelTwo)
                .alongWith(new WaitUntilCommand(m_arm::isClear)
                    .andThen(m_elevator.setPositionCmd(ElevatorConstants.kLevelTwo)
                        .alongWith(m_smallpPeice.setCanShoot(true)))),
            m_arm::isClear));

    m_driverController.leftTrigger().and(m_operatorBoard.button(2))
        .whileTrue(new ConditionalCommand(
            m_elevator.setPositionCmd(ElevatorConstants.kLevelThree)
                .alongWith(m_arm.setPositionCmd(ArmConstants.kLevelThree)
                    .alongWith(m_smallpPeice.setCanShoot(true))),
            m_arm.setPositionCmd(ArmConstants.kLevelThree)
                .andThen(new WaitUntilCommand(m_arm::isClear)
                    .andThen(m_elevator.setPositionCmd(ElevatorConstants.kLevelThree)
                        .alongWith(m_smallpPeice.setCanShoot(true)))),
            m_arm::isClear));

    m_driverController.leftBumper()
        .whileTrue(new ConditionalCommand(
            m_elevator.setPositionCmd(ElevatorConstants.kHome)
                .alongWith(new WaitUntilCommand(m_elevator::isFinished)
                    .andThen(m_arm.setPositionCmd(ArmConstants.kIntake)
                        .alongWith(new WaitUntilCommand(m_arm::isFinished)
                            .andThen(m_smallpPeice.setVoltageCmd(SmallPeiceConstants.kCoastVoltage))))),
            m_arm.setPositionCmd(ArmConstants.kClear)
                .alongWith(new WaitUntilCommand(m_arm::isClear)
                    .andThen(m_elevator.setPositionCmd(ElevatorConstants.kHome)
                        .alongWith(new WaitUntilCommand(m_elevator::isFinished)
                            .andThen(m_arm.setPositionCmd(ArmConstants.kIntake)
                                .alongWith(new WaitUntilCommand(m_arm::isFinished)
                                    .andThen(m_smallpPeice.setVoltageCmd(SmallPeiceConstants.kCoastVoltage))))))),
            m_arm::isClear));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
  }
}
