// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.ElementType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StealReefAlgaeCommand extends Command {
  /** Creates a new StealReefAlgaeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  PhotonVisionSubsystem m_photonVisionSubsystem;
  ElevatorPosition elevatorPosition;
  Timer timer;
  Timer timer2;
  boolean isFinished;
  boolean grabbedAlgae;
  int id;
  public StealReefAlgaeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
    timer = new Timer();
    timer2 = new Timer();
    isFinished = false;
    this.id = id;
    if (id % 2 == 1){
      elevatorPosition = ElevatorPosition.LOWESTALGAE;
    }
    else if (id % 2 == 0){
      elevatorPosition = ElevatorPosition.MIDALGAE;
    }
    isFinished = false;
    grabbedAlgae = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    grabbedAlgae = false;
    if (!m_photonVisionSubsystem.getIsRedAlliance()){
      if (id == 21){
        id = 10;
      }
      else if (id == 20){
        id = 11;
      }
      else if (id == 19){
        id = 6;
      }
      else if (id == 18){
        id = 7;
      }
      else if (id == 17){
        id = 8;
      }
      else if (id == 22){
        id = 9;
      }
    }
    
    m_photonVisionSubsystem.resetCount();
    m_photonVisionSubsystem.enableAlign(true, 0, 0.35, id);  
    m_clawSubsystem.setClawTargetPosition(ClawPosition.REVERSEFACINGUPALGAE);
    m_clawSubsystem.setDriveMotor(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_photonVisionSubsystem.getStealAlgaeAtTargetPosition()){
      m_elevatorSubsystem.setElevatorTargetPosition(elevatorPosition);
    }
    if (m_clawSubsystem.getDriveMotorCurrent()>59){
      timer.start();
    }
    if (timer.get()>0.3 && m_clawSubsystem.getDriveMotorCurrent()>59){
      grabbedAlgae = true;
      m_clawSubsystem.setHasAlgae(true);
      m_clawSubsystem.singularReefAlgaeDefault();
      m_photonVisionSubsystem.enableAlign(true, 0, 0.42, id);
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
      m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
      
    }
    else if (timer.get()>0.4){
      timer.reset();
      timer.stop();
    }
    if (grabbedAlgae && m_photonVisionSubsystem.getAtTargetPosition()){
      isFinished = true;
    }
    if (m_photonVisionSubsystem.getAtTargetPosition()){
      timer2.start();
    }
    if (timer2.get()>3){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
    timer2.reset();
    timer2.stop();
    m_clawSubsystem.singularReefAlgaeDefault();
    m_clawSubsystem.setDriveMotor(0);
    m_clawSubsystem.setAlgaeRetainPosition();
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    m_photonVisionSubsystem.enableAlign(false,0,0,0);
    m_photonVisionSubsystem.setDriveCommandDisabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
