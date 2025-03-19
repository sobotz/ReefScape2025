// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabAlgaeCommand extends Command {
  /** Creates a new GrabHigherAlgaeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  PhotonVisionSubsystem m_photonVisionSubsystem;
  ElevatorPosition elevatorPosition;
  ClawPosition clawPosition;
  Timer timer;
  Timer timer2;
  boolean isFinished;
  int id;
  
  
  public GrabAlgaeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem,PhotonVisionSubsystem photonVisionSubsystem, ElevatorPosition elevatorPosition, ClawPosition clawPosition, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
    this.elevatorPosition = elevatorPosition;
    this.clawPosition = clawPosition;
    timer = new Timer();
    timer2 = new Timer();
    isFinished = false;
    this.id = id;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (m_clawSubsystem.getReefAlgaeGrabButton()){
      m_photonVisionSubsystem.resetCount();
      if (m_clawSubsystem.getReefCoralPlacementButton()){
        m_photonVisionSubsystem.enableAlign(true, 0, 0.35, id);
      }
      else{
        m_photonVisionSubsystem.enableAlign(true, 0, 0.38, id);
      }
      
      if (m_clawSubsystem.getReefCoralPlacementButton()){
        if (m_elevatorSubsystem.getPositionMap().get(m_elevatorSubsystem.getAutoPlacePosition()) > m_elevatorSubsystem.getPositionMap().get(elevatorPosition)){
          clawPosition = ClawPosition.FACINGDOWNREEFALGAE;
       }
        else{
          clawPosition = ClawPosition.FACINGUPREEFALGAE;
          if (elevatorPosition == ElevatorPosition.HIGHERALGAE){
            elevatorPosition = ElevatorPosition.MIDALGAE;
          }
        }
      }
      else if (m_clawSubsystem.getReefAlgaeGrabButton() && !m_clawSubsystem.getReefCoralPlacementButton()){
        if (elevatorPosition == ElevatorPosition.HIGHERALGAE){
          elevatorPosition = ElevatorPosition.MIDALGAE;
        }
        else if (elevatorPosition == ElevatorPosition.LOWERALGAE){
          elevatorPosition = ElevatorPosition.LOWESTALGAE;
        }
        clawPosition = ClawPosition.REVERSEFACINGUPALGAE;
      }
      
      
      
      m_clawSubsystem.setClawTargetPosition(clawPosition);
      m_clawSubsystem.setDriveMotor(1);
      timer.start();
    }
    else{
      isFinished = true;
    }
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonVisionSubsystem.getAtTargetPosition()){
      m_elevatorSubsystem.setElevatorTargetPosition(elevatorPosition);
    }
    if (m_clawSubsystem.getDriveMotorCurrent()>59){
      timer2.start();
    }
    if (timer.get()>3){
      isFinished = true;
    }
    if (timer2.get()>0.35 && m_clawSubsystem.getDriveMotorCurrent()>59){
      m_clawSubsystem.setHasAlgae(true);
      if (!m_clawSubsystem.getReefCoralPlacementButton()){
        m_clawSubsystem.singularReefAlgaeDefault();
      }
      
      isFinished = true;
    }
    else if (timer2.get()>0.5){
      timer2.reset();
      timer2.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_clawSubsystem.getReefCoralPlacementButton()){
      m_clawSubsystem.singularReefAlgaeDefault();
    }
    
    m_clawSubsystem.setDriveMotor(0);
    m_clawSubsystem.setAlgaeRetainPosition();
    timer.reset();
    timer.stop();
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
