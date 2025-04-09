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
public class AutoBargeCommand extends Command {
  /** Creates a new AutoBargeCommand. */
  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  boolean isFinished;
  Timer timer;
  public AutoBargeCommand(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    isFinished = false;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    m_clawSubsystem.setDriveMotor(-1);
    m_elevatorSubsystem.configureMotionMagic(true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()>0.4){
      m_swerveSubsystem.setDisableDrive(false);
      m_clawSubsystem.setDriveMotor(0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_clawSubsystem.setHasAlgae(false);
    timer.reset();
    timer.stop();
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
