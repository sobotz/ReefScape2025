// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralPlacementSequenceCommand extends SequentialCommandGroup {
  /** Creates a new CoralPlacementSequenceCommand. */
  ElevatorPosition elevatorPosition;
  ClawPosition clawPosition;
  public CoralPlacementSequenceCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    elevatorPosition = elevatorSubsystem.getAutoPlacePosition();
    clawPosition = clawSubsystem.getAutoPlacePosition();
    addCommands(new SetActuatorPositionCommand(elevatorSubsystem, clawSubsystem, elevatorPosition, clawPosition),
      new ClawDriveCommand(clawSubsystem)
      );
  }
}
