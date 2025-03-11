// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {
  /** Creates a new AlignCommand. */
  SwerveSubsystem m_swerveSubsystem;
  ClawSubsystem m_clawSubsystem;
  PhotonVisionSubsystem m_photonVisionSubsystem;
  boolean alignActive;
  double xTarget;
  double yTarget;
  int id;
  boolean isFinished;
  boolean isAuto;
  public AlignCommand(SwerveSubsystem swerveSubsystem,ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem, boolean align, double x, double y, int id, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    alignActive = align;
    m_swerveSubsystem = swerveSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
    xTarget = x;
    yTarget = y;
    this.id = id;
    isFinished = false;
    this.isAuto = isAuto;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isAuto || m_clawSubsystem.getReefAlgaeGrabButton()){
      m_swerveSubsystem.resetCount();
      m_photonVisionSubsystem.enableAlign(alignActive ,xTarget, yTarget, id);
      isFinished = false;
    }
    else if (isAuto){
      isFinished = true;
    }
    if (!alignActive){
      m_photonVisionSubsystem.enableAlign(false,0,0,0);
      m_swerveSubsystem.setDriveCommandDisabled(false);
      m_photonVisionSubsystem.align(0,0,0,false);
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("isAligning");
    if (m_photonVisionSubsystem.getAtTargetPosition()){

      //System.out.println("FINISH ALIGNNNNN");
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
