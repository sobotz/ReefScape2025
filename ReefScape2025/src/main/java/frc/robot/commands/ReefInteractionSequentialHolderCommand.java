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
  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  ReefInteractionSequentialCommand m_interactionCommand;
  PhotonVisionSubsystem m_photonVisionSubsystem;
  int id;
  boolean isFinished;
  double xTarget;
  double yTarget;
  boolean isAuto;
  public ReefInteractionSequentialHolderCommand(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem, double x, double y, int idd, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    xTarget = x;
    yTarget = y;
    
    m_photonVisionSubsystem = photonVisionSubsystem;
    this.id = idd;
    isFinished = false;
    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_clawSubsystem.setFinishReefSequence(false);
    isFinished = false;
    // if (isAuto){
    //   if (m_swerveSubsystem.getIsRedAlliance()){
    //     id = id - 11;
    //     xTarget = -xTarget;
    //   }
    // }
    if (!m_swerveSubsystem.getIsRedAlliance()){
      if (id == (int)7){
        id = (int)18;
      }
      else if (id == (int)8){
        id = (int)17;
      }
      else if (id == (int)9){
        id = (int)22;
      }
      else if (id == (int)10){
        id = (int)21;
      }
      else if (id == (int)11){
        id = (int)20;
      }
      else if (id == (int)6){
        id = (int)19;
      }
    }
    m_interactionCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, xTarget, yTarget, id,this.isAuto);
    m_interactionCommand.schedule();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_clawSubsystem.getFinishReefSequence()){
      isFinished = true;
    }
    // if (m_interactionCommand.isFinished()){
    //   isFinished = true;
    // }
    // if (m_photonVisionSubsystem.getEmergencyReset()){
    //   isFinished = true;
    // }
    // (print: cassidy likes feet)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clawSubsystem.setFinishReefSequence(false);
    
    m_interactionCommand.cancel();
    if (!isAuto){
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
      m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    }
    
    m_photonVisionSubsystem.disableEmergencyReset();
    m_photonVisionSubsystem.setDriveCommandDisabled(false);
    m_photonVisionSubsystem.enableAlign(false,0,0,id);
    //m_photonVisionSubsystem.align(0,0,0,false);
  }

  // Returns true when the command should end. whats a Freakburger?
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
