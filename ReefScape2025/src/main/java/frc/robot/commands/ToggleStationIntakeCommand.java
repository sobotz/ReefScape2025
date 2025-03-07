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
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleStationIntakeCommand extends Command {
  /** Creates a new ToggleIntakeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  Timer timer;
  boolean isFinished;
  public ToggleStationIntakeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    timer = new Timer();
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (!m_clawSubsystem.hasItem()){
      timer.start();
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.INTAKE);
      m_clawSubsystem.setClawTargetPosition(ClawPosition.INTAKE);
      m_clawSubsystem.setDriveMotor(1);///??????CHANGE
      m_intakeSubsystem.setDriveMotor(-1);
    }
    else{
      isFinished = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_elevatorSubsystem.elevatorAtTargetPosition() && m_clawSubsystem.clawAtTargetPosition()){
      m_intakeSubsystem.setDriveMotor(1.0);
    }
    else if (timer.get()>1){
      m_intakeSubsystem.setDriveMotor(1.0);
    }
    // if (m_clawSubsystem.getProximityTripped()){
    //   m_clawSubsystem.setHasCoral(true);
    //   m_clawSubsystem.setHasAlgae(false);
    //   isFinished = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clawSubsystem.setHasCoral(true);
      m_clawSubsystem.setHasAlgae(false);
      isFinished = true;
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    m_clawSubsystem.setDriveMotor(0);
    m_intakeSubsystem.setDriveMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
