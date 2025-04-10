// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPrepBargeCommand extends Command {
  /** Creates a new AutoPrepBargeCommand. */
  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  Boolean isFinished;
  public AutoPrepBargeCommand(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSubsystem.drive(new Vector(0, 0), 0, 0, false, true);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.BARGE);
    if (m_clawSubsystem.getPositionMap().get(ClawPosition.DEFAULT) == 0){
      m_clawSubsystem.setClawTargetPosition(ClawPosition.BARGE2);
    }
    else{
      m_clawSubsystem.setClawTargetPosition(ClawPosition.BARGE);
    }
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_swerveSubsystem.setDisableDrive(true);
    if (m_elevatorSubsystem.elevatorAtTargetPosition() && m_clawSubsystem.clawAtTargetPosition()){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
