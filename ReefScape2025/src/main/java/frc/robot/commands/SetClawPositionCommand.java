// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawPosition;
import frc.robot.subsystems.ClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClawPositionCommand extends Command {
  /** Creates a new SetClawPositionCommand. */
  ClawSubsystem m_clawSubsystem;
  ClawPosition targetPosition;
  boolean isFinished;
  Timer timer;
  public SetClawPositionCommand(ClawSubsystem clawSubsystem, ClawPosition targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_clawSubsystem = clawSubsystem;
    this.targetPosition = targetPosition;
    isFinished = false;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    m_clawSubsystem.setClawTargetPosition(targetPosition);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_clawSubsystem.clawAtTargetPosition()){
      isFinished = true;
    }
    if (timer.get()>5){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("claw Finished");
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
