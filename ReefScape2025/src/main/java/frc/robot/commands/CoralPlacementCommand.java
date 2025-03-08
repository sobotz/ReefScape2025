// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPlacementCommand extends Command {
  /** Creates a new CoralPlacementCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  PhotonVisionSubsystem m_photonVisionSubsystem;
  boolean isFinished;
  boolean alignEnabled;
  double xTarget;
  double yTarget;
  int id;
  Timer timer;
  Timer timer2;
  public CoralPlacementCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem,PhotonVisionSubsystem photonVisionSubsystem, boolean align, double x, double y, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
    isFinished = false;
    xTarget = x;
    yTarget = y;
    this.id = id;

    timer = new Timer();
    timer2 = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (m_clawSubsystem.getReefCoralPlacementButton()){
      System.out.println("start coral placement");
      //m_photonVisionSubsystem.resetCount();
      //m_photonVisionSubsystem.enableAlign(true, xTarget, yTarget, id);
      m_elevatorSubsystem.setElevatorTargetPosition(m_elevatorSubsystem.getAutoPlacePosition());
      m_clawSubsystem.setClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
      timer.start();
      if (m_elevatorSubsystem.getAutoPlacePosition() == ElevatorPosition.L3 || m_elevatorSubsystem.getAutoPlacePosition() == ElevatorPosition.L4){
        //m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.TEMPPOSITION);
        //m_clawSubsystem.setClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
      }
    }
    else{
      isFinished = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_photonVisionSubsystem.getAtTargetPosition()){
    //   m_elevatorSubsystem.setElevatorTargetPosition(m_elevatorSubsystem.getAutoPlacePosition());
    //   m_clawSubsystem.setClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
    //   timer.start();
    // }
    if (m_clawSubsystem.clawAtTargetPosition() && m_elevatorSubsystem.elevatorAtTargetPosition()){
      m_clawSubsystem.setDriveMotor(-0.6);
      timer2.start();
    }
    else if(timer.get()>4){
      timer2.start();
      m_clawSubsystem.setDriveMotor(-0.6);
    }
    if (timer2.get()>0.4){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clawSubsystem.setHasCoral(false);
    timer.reset();
    timer2.reset();
    timer.stop();
    timer2.stop();
    m_clawSubsystem.setDriveMotor(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
