// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefInteractionSequentialHolderCommand extends Command {
  /** Creates a new ReefInteractionSequentialHolderCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  ReefInteractionSequentialCommand m_interactionCommand;
  PhotonVisionSubsystem m_photonVisionSubsystem;
  int id;
  boolean isFinished;
  public ReefInteractionSequentialHolderCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem, int id, ReefInteractionSequentialCommand interactionCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_interactionCommand = interactionCommand;
    m_photonVisionSubsystem = photonVisionSubsystem;
    this.id = id;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    m_interactionCommand.schedule();
    /*if (m_photonVisionSubsystem.hasRightID(id)){
      m_interactionCommand.schedule();
    }
    else{
      isFinished = true;
    }*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_interactionCommand.isFinished()){
      isFinished = true;
    }
    if (m_photonVisionSubsystem.getEmergencyReset()){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_interactionCommand.cancel();
    //m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    //m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    m_photonVisionSubsystem.disableEmergencyReset();
    m_photonVisionSubsystem.setDriveCommandDisabled(false);
    m_photonVisionSubsystem.enableAlign(false,0,0,id);
    //m_photonVisionSubsystem.align(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
