// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPlacementCommand extends Command {
  /** Creates a new CoralPlacementCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  boolean isFinished;
  Timer timer;
  Timer timer2;
  public CoralPlacementCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    isFinished = false;
    timer = new Timer();
    timer2 = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (m_clawSubsystem.getReefCoralPlacementButton()){
      m_elevatorSubsystem.setElevatorTargetPosition(m_elevatorSubsystem.getAutoPlacePosition());
      m_clawSubsystem.setAutoPlaceClawTargetPosition(m_clawSubsystem.getAutoPlacePosition());
      timer.start();
    }
    else{
      isFinished = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_clawSubsystem.clawAtTargetPosition() && m_elevatorSubsystem.elevatorAtTargetPosition()){
      m_clawSubsystem.setDriveMotor(-0.8);
      timer2.start();
    }
    else if(timer.get()>3){
      timer2.start();
      m_clawSubsystem.setDriveMotor(-0.8);
    }
    if (timer2.get()>0.8){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
