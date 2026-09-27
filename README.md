# SubsystemManager

A WPILib command-based FRC robot project demonstrating coordinated control of an arm, elevator, and small-piece manipulator. The arm and elevator use CTRE Phoenix 6 Motion Magic position requests. The manipulator uses voltage control.

## Controls

| Input | Behavior |
| --- | --- |
| Driver right trigger | Score with the small-piece manipulator while the arm and elevator are at a selected scoring level. Holding the trigger before the mechanisms arrive waits at zero volts, then scores when they reach the level. |
| Driver left trigger + operator board button 1 | Move to level one. |
| Driver left trigger + operator board button 2 | Move to level two. |
| Driver left trigger + operator board button 3 | Move to level three. |
| Driver left bumper | Bring the elevator home, move the arm to intake, then run the manipulator at coast voltage while held. |

The operator board is on USB port 1; the Xbox controller is on USB port 0. WPILib HID button numbers start at 1. Check the button numbers in Driver Station against the physical operator board before using the controls.

## Movement sequence

- When selecting a level, the elevator waits for the arm to reach its positive-side clearance position if the arm is not already clear. The arm and elevator then move to the chosen level together.
- Scoring is permitted only while a level command is held and both mechanisms are within their position tolerances for that level. Permission updates as their positions change and clears when the level command ends.
- On the way to intake, the elevator reaches home before the arm moves to intake. If the arm is between its defined clear regions, it first moves to the positive-side clearance position.
- Ending an arm or elevator position command leaves its last position request in place. Releasing the intake button stops the manipulator.

The clearance conditions are based on arm position. Verify the clear positions, encoder direction, position tolerances, and travel path against the actual robot before enabling both mechanisms. The current constants are project values and have not been validated on hardware.

## Project layout

- `src/main/java/frc/robot/RobotContainer.java`: controller bindings and coordinated commands.
- `src/main/java/frc/robot/subsystems/Arm.java`: arm position control and clearance checks.
- `src/main/java/frc/robot/subsystems/Elevator.java`: elevator position control.
- `src/main/java/frc/robot/subsystems/Manipulator/SmallPeice.java`: intake and gated scoring voltage.
- `src/main/java/frc/robot/Constants/`: CAN IDs, motion parameters, positions, and voltages.

`LargePeice` exists as a separate subsystem but is not currently bound to a controller. Autonomous currently returns a zero-second wait command.

## Build

Install the 2026 WPILib development tools and open this folder in the WPILib VS Code environment, or run:

```sh
./gradlew build
```

On Windows, use `gradlew.bat build`. Dependencies include GradleRIO 2026.2.1 and the versions listed in `vendordeps/`. This repository has no automated tests. A build and a hardware check are still needed before running the mechanisms; the Gradle distribution was unavailable in the environment where the command-flow changes were prepared.
