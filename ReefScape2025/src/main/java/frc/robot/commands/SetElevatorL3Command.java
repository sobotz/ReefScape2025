// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorL3Command extends InstantCommand {
    ElevatorSubsystem m_elavatorSubsystem;
  public SetElevatorL3Command(ElevatorSubsystem elavatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elavatorSubsystem = elavatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elavatorSubsystem.setElevatorTargetPosition(ElevatorPosition.L3);
  }
}
