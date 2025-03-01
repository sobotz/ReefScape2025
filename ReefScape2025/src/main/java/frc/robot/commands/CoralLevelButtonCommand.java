// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralLevelButtonCommand extends InstantCommand {
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  ElevatorPosition elevatorPosition;
  ClawPosition clawPosition;
  public CoralLevelButtonCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, double level) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    if (level == 1){
      elevatorPosition = ElevatorPosition.L1;
      clawPosition = ClawPosition.L1;
    }
    else if (level == 2){
      elevatorPosition = ElevatorPosition.L2;
      clawPosition = ClawPosition.L2;
    }
    else if (level == 3){
      elevatorPosition = ElevatorPosition.L3;
      clawPosition = ClawPosition.L3;
    }
    else if (level == 4){
      elevatorPosition = ElevatorPosition.L4;
      clawPosition = ClawPosition.L4;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setAutoPlaceClawTargetPosition(elevatorPosition);
    m_clawSubsystem.setAutoPlaceClawTargetPosition(clawPosition);
  }
}
