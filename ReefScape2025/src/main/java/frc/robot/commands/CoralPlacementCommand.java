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
  boolean isAuto;
  boolean once;
  public CoralPlacementCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem,PhotonVisionSubsystem photonVisionSubsystem, boolean align, double x, double y, int id, boolean auto) {
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
    isAuto = auto;
    once = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    //System.out.println("placecoral button: " + m_clawSubsystem.getReefCoralPlacementButton());
    
    if (m_clawSubsystem.getReefCoralPlacementButton()){
      m_photonVisionSubsystem.resetCount();
      // if (xTarget < 0 && m_clawSubsystem.getAutoPlacePosition() == ClawPosition.L3){
      //   xTarget += 0.02;
      // }
      // if (xTarget > 0 && m_clawSubsystem.getAutoPlacePosition() == ClawPosition.L3){
      //   xTarget -= 0.02;
      // }
      m_photonVisionSubsystem.enableAlign(true, xTarget, yTarget, id);
      // if (isAuto){
      //   m_elevatorSubsystem.setAutoPlaceClawTargetPosition(ElevatorPosition.L4);
      //   m_clawSubsystem.setAutoPlaceClawTargetPosition(ClawPosition.L4);
      // }
      m_elevatorSubsystem.setElevatorTargetPosition(m_elevatorSubsystem.getAutoPlacePosition());
      if (!(m_clawSubsystem.getAutoPlacePosition()== ClawPosition.L4)){
        System.out.println("WRONG");
        m_clawSubsystem.setClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
      }
      //m_clawSubsystem.setClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
      timer.start();
    }
    else{
      isFinished = true;
    }
    // if (isAuto){
    //   m_elevatorSubsystem.configureMotionMagic(false);
    // }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_clawSubsystem.getAutoPlacePosition()== ClawPosition.L4) && (m_elevatorSubsystem.getElevatorSensorPosition()>42)){
      m_clawSubsystem.setClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
    }
    // if (m_photonVisionSubsystem.getAtTargetPosition()){
    //   m_elevatorSubsystem.setElevatorTargetPosition(m_elevatorSubsystem.getAutoPlacePosition());
    //   m_clawSubsystem.setClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
    //   timer.start();
    // }
    //System.out.println("RUNNING");
    
    if ((m_clawSubsystem.clawAtTargetPosition() && m_elevatorSubsystem.elevatorAtTargetPosition()) && (m_photonVisionSubsystem.getAtTargetPosition() && (m_clawSubsystem.getAutoPlacePosition() == m_clawSubsystem.getTargetPosition()))){
      if (m_clawSubsystem.getAutoPlacePosition() == ClawPosition.L3 || m_clawSubsystem.getAutoPlacePosition() == ClawPosition.L2){
        m_clawSubsystem.setDriveMotor(-0.34);
      }
      else{
        m_clawSubsystem.setDriveMotor(-0.6);
      }
      if (isAuto){
        if (once){
          m_elevatorSubsystem.configureMotionMagic(true);
        }
      }
      timer2.start();
    }
    else if(timer.get()>4){
      timer2.start();
      m_clawSubsystem.setDriveMotor(-0.6);
    }
    // if (!m_photonVisionSubsystem.getHasTarget()){
    //   timer.restart();
    // }
    if (m_clawSubsystem.getAutoPlacePosition() == ClawPosition.L3 || m_clawSubsystem.getAutoPlacePosition() == ClawPosition.L2){
      if (timer2.get()>0.6){
        isFinished = true;
      }
    }
    else{
      if (timer2.get()>0.4){
        isFinished = true;
      }
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
