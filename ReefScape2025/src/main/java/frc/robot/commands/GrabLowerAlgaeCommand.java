// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabLowerAlgaeCommand extends Command {
  /** Creates a new GrabLowerAlgaeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  public GrabLowerAlgaeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.LOWERALGAE);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.FACINGDOWNREEFALGAE);
    m_clawSubsystem.setDriveMotor(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clawSubsystem.setHasAlgae(true);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    m_clawSubsystem.setDriveMotor(0);
    m_clawSubsystem.setAlgaeRetainPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
