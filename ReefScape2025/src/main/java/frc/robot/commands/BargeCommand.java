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
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeCommand extends Command {
  /** Creates a new BargeCommand. */
  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  Timer timer;
  boolean isFinished;
  public BargeCommand(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    timer = new Timer();
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (m_clawSubsystem.getHasAlgae()){
      m_clawSubsystem.toggleBarge();
      if (m_clawSubsystem.getToggleBarge() == true){
        m_swerveSubsystem.setBargeMode(true);
         m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.BARGE);
        m_clawSubsystem.setClawTargetPosition(ClawPosition.BARGE);
        isFinished = true;
      }
      else{
        m_clawSubsystem.setDriveMotor(-0.5);
        timer.start();
      }
    }
    else{
      isFinished = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()>0.4){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
    if (m_clawSubsystem.getToggleBarge() == false){
      m_clawSubsystem.setHasAlgae(false);
      m_swerveSubsystem.setBargeMode(false);
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
      m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
