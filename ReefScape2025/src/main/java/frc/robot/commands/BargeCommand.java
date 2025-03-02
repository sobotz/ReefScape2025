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
public class BargeCommand extends Command {
  /** Creates a new BargeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  Timer timer;
  Timer timer2;
  boolean isFinished;
  public BargeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    timer = new Timer();
    timer2 = new Timer();
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.BARGE);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.BARGE);
    timer.start();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (m_elevatorSubsystem.elevatorAtTargetPosition() && m_clawSubsystem.clawAtTargetPosition()){
      m_clawSubsystem.setDriveMotor(0.5);
      timer2.start();
    }
    else if(timer.get()>4){
      timer2.start();
      m_clawSubsystem.setDriveMotor(0.5);
    }
    if (timer2.get()>0.5){
      isFinished = true;
    }*/
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
    timer2.reset();
    timer2.stop();
    m_clawSubsystem.setHasAlgae(false);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    m_clawSubsystem.setDriveMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
