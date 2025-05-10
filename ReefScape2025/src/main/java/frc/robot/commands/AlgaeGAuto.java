// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeGAuto extends SequentialCommandGroup {
  /** Creates a new AlgaeGAuto. */
  public AlgaeGAuto(
    AutoFactory autoFactory,
    SwerveSubsystem swerveSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    ClawSubsystem clawSubsystem,
    PhotonVisionSubsystem photonVisionSubsystem
    ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      autoFactory.resetOdometry("Start-Goffset"),
      autoFactory.trajectoryCmd("Start-Goffset"),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, -0.162, 0.435, 10, true),
      autoFactory.trajectoryCmd("Algae21-Barge1"),
      new ResetElevatorConfigCommand(elevatorSubsystem),
      new AutoPrepBargeCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem),
      new AutoBargeCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem),
      autoFactory.trajectoryCmd("Barge1-Algae20"),
      new ResetElevatorConfigCommand(elevatorSubsystem),
      new ReefCoralPlacementButton(clawSubsystem),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, -0.162, 0.435, 11, true),
      autoFactory.trajectoryCmd("Algae20-Barge1"),
      new ResetElevatorConfigCommand(elevatorSubsystem),
      new AutoPrepBargeCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem),
      new AutoBargeCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem),
      autoFactory.trajectoryCmd("Barge1-OffLine"),
      new ResetElevatorConfigCommand(elevatorSubsystem)
    );
  }
}
