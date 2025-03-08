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


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabAlgaeCommand extends Command {
  /** Creates a new GrabHigherAlgaeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  ElevatorPosition elevatorPosition;
  ClawPosition clawPosition;
  Timer timer;
  Timer timer2;
  boolean isFinished;
  
  
  public GrabAlgaeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, ElevatorPosition elevatorPosition, ClawPosition clawPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    this.elevatorPosition = elevatorPosition;
    this.clawPosition = clawPosition;
    timer = new Timer();
    timer2 = new Timer();
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (m_clawSubsystem.getReefAlgaeGrabButton()){
      if (m_elevatorSubsystem.getPositionMap().get(m_elevatorSubsystem.getAutoPlacePosition()) > m_elevatorSubsystem.getPositionMap().get(elevatorPosition)){
        clawPosition = ClawPosition.FACINGDOWNREEFALGAE;
      }
      else{
        clawPosition = ClawPosition.FACINGUPREEFALGAE;
        if (elevatorPosition == ElevatorPosition.HIGHERALGAE){
          elevatorPosition = ElevatorPosition.MIDALGAE;
        }
      }
      m_elevatorSubsystem.setElevatorTargetPosition(elevatorPosition);
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
    if (m_clawSubsystem.getDriveMotorCurrent()>60){
      timer2.start();
    }
    if (timer.get()>3){
      isFinished = true;
    }
    if (timer2.get()>0.35 && m_clawSubsystem.getDriveMotorCurrent()>60){
      m_clawSubsystem.setHasAlgae(true);
      m_clawSubsystem.setAlgaeRetainPosition();
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
