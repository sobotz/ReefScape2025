// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {
  /** Creates a new AlignCommand. */
  PhotonVisionSubsystem m_photonVisionSubsystem;
  boolean alignActive;
  double xTarget;
  double yTarget;
  int id;
  boolean isFinished;
  public AlignCommand(PhotonVisionSubsystem photonVisionSubsystem, boolean align, double x, double y, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    alignActive = align;
    m_photonVisionSubsystem = photonVisionSubsystem;
    xTarget = x;
    yTarget = y;
    this.id = id;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_photonVisionSubsystem.enableAlign(alignActive ,xTarget, yTarget, id);
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonVisionSubsystem.getAtTargetPosition()){
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
