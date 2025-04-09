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
public class InstantDefaultCommand extends InstantCommand {
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  Boolean isAuto;
  public InstantDefaultCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!(m_clawSubsystem.getReefAlgaeGrabButton() && isAuto)){
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
      m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    }
    
  }
}
