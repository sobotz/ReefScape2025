// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PhotonVisionCommand extends Command {
  /** Creates a new PhotonVisionCommand. */
  PhotonVisionSubsystem m_PhotonVisionSubsytem;
  SwerveSubsystem m_SwerveSubsystem;
  public PhotonVisionCommand(PhotonVisionSubsystem photonsubsytem,SwerveSubsystem swervesubsystem) {
    m_PhotonVisionSubsytem = photonsubsytem;
    m_SwerveSubsystem = swervesubsystem;

    addRequirements(m_PhotonVisionSubsytem,m_SwerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_PhotonVisionSubsytem.align(-0.3,0.4,20,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PhotonVisionSubsytem.align(0,0.5,20,false);
    m_SwerveSubsystem.setDriveCommandDisabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
