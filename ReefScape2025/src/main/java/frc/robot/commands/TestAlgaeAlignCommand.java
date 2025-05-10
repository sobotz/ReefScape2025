// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAlgaeAlignCommand extends Command {
  /** Creates a new TestAlgaeAlignCommand. */
  PhotonVisionSubsystem m_photonVisionSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  public TestAlgaeAlignCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.FLOORALGAE);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.FLOORALGAE);
    m_photonVisionSubsystem.setAlgaeAlign(true);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonVisionSubsystem.getAlgaeHasTarget()){
      m_photonVisionSubsystem.setAlgaeCapturedTarget(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
    //drive(new Vector(0, 0),0,currentRobotDegree,false,false);
    m_photonVisionSubsystem.setAlgaeAlign(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
