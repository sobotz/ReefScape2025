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
public class ToggleAutoFloorAlgaeIntakeCommand extends Command {
  /** Creates a new ToggleFloorAlgaeIntakeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  PhotonVisionSubsystem m_photonVisionSubsystem;
  ClawSubsystem m_clawSubsystem;
  boolean isFinished;
  Timer timer;
  Timer timer2;
  public ToggleAutoFloorAlgaeIntakeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
    isFinished = false;
    timer = new Timer();
    timer2 = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("on");
    isFinished = false;
    m_clawSubsystem.setClawTargetPosition(ClawPosition.FLOORALGAE);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.FLOORALGAE);
    m_photonVisionSubsystem.setAlgaeAlign(true);
    m_clawSubsystem.setDriveMotor(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_clawSubsystem.clawAtTargetPosition()){
    //   m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.FLOORALGAE);
    // }
    if (m_photonVisionSubsystem.getAlgaeHasTarget()){
      m_photonVisionSubsystem.setAlgaeCapturedTarget(true);
    }
    if(m_clawSubsystem.getDriveMotorCurrent()>60){
      timer.start();
    }
    if (timer.get()>0.3 && m_clawSubsystem.getDriveMotorCurrent()>53){
      m_photonVisionSubsystem.setDriveCommandDisabled(false);
      m_photonVisionSubsystem.setAlgaeAlign(false);
      m_clawSubsystem.setHasAlgae(true);
      m_clawSubsystem.setHasCoral(false);
      m_clawSubsystem.setAlgaeRetainPosition();
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.ALGAETEMP);
      timer2.start();
    }
    else if (timer.get()>0.5){
      timer.reset();
      timer.stop();
    }
    if (timer2.get()>0.3){
      isFinished = true;
    }
    
  }
  // Called once the command ends or is interrupted.
  @Override
  
  public void end(boolean interrupted) {
    //m_clawSubsystem.setHasAlgae(true)
    m_photonVisionSubsystem.setAlgaeAlign(false);
    m_photonVisionSubsystem.setDriveCommandDisabled(false);
    timer.reset();
    timer.stop();
    timer2.reset();
    timer2.stop();
    m_clawSubsystem.setDriveMotor(0);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
