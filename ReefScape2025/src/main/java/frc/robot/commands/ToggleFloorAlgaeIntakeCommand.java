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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleFloorAlgaeIntakeCommand extends Command {
  /** Creates a new ToggleFloorAlgaeIntakeCommand. */
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  boolean isFinished;
  Timer timer;
  public ToggleFloorAlgaeIntakeCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_clawSubsystem = clawSubsystem;
    isFinished = false;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("on");
    isFinished = false;
    
    m_clawSubsystem.setClawTargetPosition(ClawPosition.FLOORALGAE);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.FLOORALGAE);
    m_clawSubsystem.setDriveMotor(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_clawSubsystem.clawAtTargetPosition()){
    //   m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.FLOORALGAE);
    // }
    if(m_clawSubsystem.getDriveMotorCurrent()>60){
      timer.start();
    }
    if (timer.get()>0.3 && m_clawSubsystem.getDriveMotorCurrent()>59){
      m_clawSubsystem.setHasAlgae(true);
      m_clawSubsystem.setHasCoral(false);
      m_clawSubsystem.setAlgaeRetainPosition();
      m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.ALGAETEMP);
      if (m_elevatorSubsystem.elevatorAtTargetPosition() && timer.get()>0.4){
        isFinished = true;
      }
    }
    else if (timer.get()>0.6){
      timer.reset();
      timer.stop();
    }
    
  }
  // Called once the command ends or is interrupted.
  @Override
  
  public void end(boolean interrupted) {
    //m_clawSubsystem.setHasAlgae(true)
    timer.reset();
    timer.stop();
    m_clawSubsystem.setDriveMotor(0);
    m_elevatorSubsystem.setElevatorTargetPosition(ElevatorPosition.DEFAULT);
    m_clawSubsystem.setClawTargetPosition(ClawPosition.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
