// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleClimbCommand extends InstantCommand {
  IntakeSubsystem m_intakeSubsystem;
  ClimbSubsystem m_climbSubsystem;
  public ToggleClimbCommand(IntakeSubsystem intakeSubsystem, ClimbSubsystem climbSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intakeSubsystem;
    m_climbSubsystem = climbSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.openServo();
    m_climbSubsystem.toggleClimb();
  }
}
