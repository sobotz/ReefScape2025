// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetActuatorPositionCommand extends ParallelCommandGroup {
  /** Creates a new SetActuatorPositionCommand. */
  public SetActuatorPositionCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, ElevatorPosition elevatorTargetPosition, ClawPosition clawTargetPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorPositionCommand(elevatorSubsystem, elevatorTargetPosition), new SetClawPositionCommand(clawSubsystem, clawTargetPosition));
  }
}
