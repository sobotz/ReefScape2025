// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPositionCommand extends Command {
  /** Creates a new SetElevatorPositionCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ElevatorPosition targetPosition;
  boolean isFinished;
  Timer timer;
  public SetElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPosition targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    this.targetPosition = targetPosition;
    isFinished = false;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    isFinished = false;
    m_elevatorSubsystem.setElevatorTargetPosition(targetPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_elevatorSubsystem.elevatorAtTargetPosition()){
      isFinished = true;
    }
    if (timer.get()>5){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("Elevator Finished");
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
