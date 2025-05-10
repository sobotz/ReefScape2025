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
public class IntakeStartUpCommand extends InstantCommand {
  ClawSubsystem m_clawSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  public IntakeStartUpCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    m_clawSubsystem.enableStartUp();
    if (!m_clawSubsystem.getHasAlgae() && m_clawSubsystem.getProximityTripped()){
      m_clawSubsystem.setHasCoral(true);
    }
    else{
      m_clawSubsystem.setHasCoral(false);
    }
    m_clawSubsystem.setDriveMotor(0);
    
  }
}
