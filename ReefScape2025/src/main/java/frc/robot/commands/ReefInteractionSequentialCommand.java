// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReefInteractionSequentialCommand extends SequentialCommandGroup {
  /** Creates a new ReefInteractionSequentialCommand. */
  Command algaeGrabCommand;
  Map<ElevatorPosition, Double> elevatorPositionMap;
  Map<ClawPosition, Double> clawPositionMap;
  ElevatorPosition algaeElevatorPosition;
  ClawPosition algaeClawPosition;
  
  int id;
  public ReefInteractionSequentialCommand(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem,PhotonVisionSubsystem photonVisionSubsystem,double xTarget, double yTarget, int id) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //elevatorSubsystem.setAutoPlaceClawTargetPosition(targetElevatorPosition);
    //clawSubsystem.setAutoPlaceClawTargetPosition(targetClawPosition);
    elevatorPositionMap = elevatorSubsystem.getPositionMap();
    clawPositionMap = clawSubsystem.getPositionMap();
    this.id = id;
    if (id>=17 && id<=22){
      if (id % 2 == 0){
        algaeElevatorPosition = ElevatorPosition.HIGHERALGAE;
      }
      else{
        algaeElevatorPosition = ElevatorPosition.LOWERALGAE;
      }
    }
    else if (id>=6 && id<=11){
      if (id % 2 == 0){
        algaeElevatorPosition = ElevatorPosition.LOWERALGAE;
      }
      else{
        algaeElevatorPosition = ElevatorPosition.HIGHERALGAE;
      }
    }
    if (elevatorPositionMap.get(elevatorSubsystem.getAutoPlacePosition()) > elevatorPositionMap.get(algaeElevatorPosition)){
      algaeClawPosition = ClawPosition.FACINGDOWNREEFALGAE;
    }
    else{
      algaeClawPosition = ClawPosition.FACINGUPREEFALGAE;
    }
    //System.out.println("id: " + id);
    addCommands(
      new AlignCommand(swerveSubsystem, photonVisionSubsystem, true, xTarget, yTarget, id),
      new CoralPlacementCommand(elevatorSubsystem, clawSubsystem),
      new ParallelCommandGroup(
        new GrabAlgaeCommand(elevatorSubsystem, clawSubsystem, algaeElevatorPosition, algaeClawPosition),
        new AlignCommand(swerveSubsystem ,photonVisionSubsystem, true, 0, 0.36, id)
      ),
      new AlignCommand(swerveSubsystem, photonVisionSubsystem, true, 0, 0.43, id),
      new SetActuatorPositionCommand(elevatorSubsystem, clawSubsystem, ElevatorPosition.DEFAULT, ClawPosition.DEFAULT),
      new AlignCommand(swerveSubsystem, photonVisionSubsystem, false, 0, 0.41, id)
    );
  }
}
