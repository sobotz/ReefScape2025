// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetClawCommand extends Command {
  /** Creates a new ResetClawCommand. */
  Joystick a1;
  ClawSubsystem m_clawSubsystem;
  ElevatorSubsystem m_ElevatorSubsystem;
  boolean once;
  Timer timer;

  public ResetClawCommand(ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Joystick stick) {
    m_ElevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    once = false;
    a1 = stick;
    timer = new Timer();
    timer.reset();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.ALGAETEMP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (a1.getRawAxis(1) != 0) {
      once = true;
      m_clawSubsystem.setResetClaw(true);
      m_clawSubsystem.setWristMotor(-0.07);
    } else if (once) {
      once = false;
      //m_clawSubsystem.setResetClaw(false);
      timer.start();
      m_ElevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.)
      m_clawSubsystem.setWristMotor(-0.1);
      if (timer.get()>0.25){

      }

     // m_clawSubsystem.setWristMotor(0);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
