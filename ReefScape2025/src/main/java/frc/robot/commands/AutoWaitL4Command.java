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
public class AutoWaitL4Command extends Command {
  /** Creates a new AutoWaitL4Command. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  Timer timer;
  boolean isFinished;
  double time;
  public AutoWaitL4Command(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem,double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    isFinished = false;
    timer = new Timer();
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()>time){
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.L4);
      m_clawSubsystem.setClawTargetPosition(ClawPosition.L4);
      if (m_elevatorSubsystem.elevatorAtTargetPosition() && m_clawSubsystem.clawAtTargetPosition()){
        isFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
