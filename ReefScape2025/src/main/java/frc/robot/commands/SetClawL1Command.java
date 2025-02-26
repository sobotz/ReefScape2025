// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.subsystems.ClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClawL1Command extends Command {
  /** Creates a new setClawL1Command. */
  private ClawSubsystem m_ClawSubsystem;
  private boolean isFinished = false;
  public SetClawL1Command(ClawSubsystem clawSubsystem) {
    m_ClawSubsystem = clawSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    m_ClawSubsystem.setClawTargetPosition(ClawPosition.L1);
    //System.out.println("init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("running l1");
    if(m_ClawSubsystem.clawAtTargetPosition()){
      System.out.println("finished");
      isFinished = true;
    }
    //m_ClawSubsystem.setWristMotor(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_ClawSubsystem.setWristMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
