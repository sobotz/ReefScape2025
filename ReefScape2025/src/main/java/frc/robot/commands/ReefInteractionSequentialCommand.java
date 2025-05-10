// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
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
  PhotonVisionSubsystem m_photonVisionSubsystem;
  Map<ElevatorPosition, Double> elevatorPositionMap;
  Map<ClawPosition, Double> clawPositionMap;
  ElevatorPosition algaeElevatorPosition;
  ClawPosition algaeClawPosition;
  boolean isRightSide;
  boolean isAuto;
  
  
  int id;
  public ReefInteractionSequentialCommand(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem,PhotonVisionSubsystem photonVisionSubsystem,double x, double y, int idd, boolean isAuto) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //elevatorSubsystem.setAutoPlaceClawTargetPosition(targetElevatorPosition);
    //clawSubsystem.setAutoPlaceClawTargetPosition(targetClawPosition);
    // if (isAuto){
    //   if (swerveSubsystem.getIsRedAlliance()){
    //     if (id == (int)18){
    //       id = (int)7;
    //     }
    //     else if (id == (int)17){
    //       id = (int)8;
    //     }
    //     else if (id == (int)22){
    //       id = (int)9;
    //     }
    //     else if (id == (int)21){
    //       System.out.println("GOOD");
    //       id = (int)10;
    //     }
    //     else if (id == (int)20){
    //       id = (int)11;
    //     }
    //     else if (id == (int)19){
    //       id = (int)6;
    //     }
    //   }
    //   else{
    //     id = idd;
    //   }
    
    id = idd;
    
    this.isAuto = isAuto;
    
    m_photonVisionSubsystem = photonVisionSubsystem;
    elevatorPositionMap = elevatorSubsystem.getPositionMap();
    clawPositionMap = clawSubsystem.getPositionMap();
    algaeElevatorPosition = ElevatorPosition.HIGHERALGAE;
    algaeClawPosition = ClawPosition.FACINGDOWNREEFALGAE;
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

    if (x > 0){
      isRightSide = true;
    }
    else{
      isRightSide = false;
    }
    //System.out.println(elevatorPositionMap.get(elevatorSubsystem.getAutoPlacePosition()));
    // if (elevatorPositionMap.get(elevatorSubsystem.getAutoPlacePosition()) > elevatorPositionMap.get(algaeElevatorPosition)){
    //   algaeClawPosition = ClawPosition.FACINGDOWNREEFALGAE;
    // }
    // else{
    //   algaeClawPosition = ClawPosition.FACINGUPREEFALGAE;
    //   if (algaeElevatorPosition == ElevatorPosition.HIGHERALGAE){
    //     algaeElevatorPosition = ElevatorPosition.MIDALGAE;
    //   }
    // }
    //System.out.println("id: " + id);
    addCommands(
      new CoralPlacementCommand(elevatorSubsystem, clawSubsystem, photonVisionSubsystem, true, x, y, id, isAuto),
      new GrabAlgaeCommand(elevatorSubsystem, clawSubsystem,photonVisionSubsystem, algaeElevatorPosition, algaeClawPosition, id,isAuto),
      new AlignCommand(swerveSubsystem,clawSubsystem, photonVisionSubsystem, true, 0, 0.462, id,isAuto),
      new InstantDefaultCommand(elevatorSubsystem, clawSubsystem,isAuto),
      new AlignCommand(swerveSubsystem,clawSubsystem, photonVisionSubsystem, false, 0, 0, 0,false));
      new FinishReefSequenceCommand(clawSubsystem);
  }
  public boolean getIsRightSide(){
    return isRightSide;
  }
}
