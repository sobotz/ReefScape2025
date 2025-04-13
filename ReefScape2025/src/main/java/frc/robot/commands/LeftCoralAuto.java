// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftCoralAuto extends SequentialCommandGroup {
  /** Creates a new LeftCoralAuto. */
  public LeftCoralAuto(AutoFactory autoFactory, SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ReefAlgaeGrabButton(clawSubsystem),
      autoFactory.resetOdometry("Start-Joffset"),
      autoFactory.trajectoryCmd("Start-Joffset"),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, 0.173, 0.435, 11, true),//J

      Commands.deadline(
        new AutoIntakeCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem),
        autoFactory.trajectoryCmd("J-Station").andThen(new AutoIntakeStopCommand(swerveSubsystem))
      ),

      new ResetElevatorConfigCommand(elevatorSubsystem),
      autoFactory.trajectoryCmd("Station-Loffset"),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, 0.173, 0.435, 6, true),//L

      Commands.deadline(
        new AutoIntakeCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem),
        autoFactory.trajectoryCmd("L-Station").andThen(new AutoIntakeStopCommand(swerveSubsystem))
      ),
      
      new ResetElevatorConfigCommand(elevatorSubsystem),
      autoFactory.trajectoryCmd("Station-Koffset"),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, -0.162, 0.435, 6, true),//K
      autoFactory.trajectoryCmd("K-Station")
    );
  }
}
