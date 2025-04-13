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
public class RightCoralAuto extends SequentialCommandGroup {
  /** Creates a new RightCoralAuto. */
  public RightCoralAuto(AutoFactory autoFactory, SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, PhotonVisionSubsystem photonVisionSubsystem,IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ReefAlgaeGrabButton(clawSubsystem),
      autoFactory.resetOdometry("Start-Eoffset"),
      autoFactory.trajectoryCmd("Start-Eoffset"),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, -0.162, 0.435, 9, true),

      Commands.deadline(
        new AutoIntakeCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem),
        autoFactory.trajectoryCmd("E-Station").andThen(new AutoIntakeStopCommand(swerveSubsystem))
      ),

      new ResetElevatorConfigCommand(elevatorSubsystem),
      autoFactory.trajectoryCmd("Station-Coffset"),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, -0.162, 0.435, 8, true),

      Commands.deadline(
        new AutoIntakeCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem),
        autoFactory.trajectoryCmd("C-Station").andThen(new AutoIntakeStopCommand(swerveSubsystem))
      ),
      
      new ResetElevatorConfigCommand(elevatorSubsystem),
      autoFactory.trajectoryCmd("Station-Doffset"),
      new ReefInteractionSequentialHolderCommand(swerveSubsystem, elevatorSubsystem, clawSubsystem, photonVisionSubsystem, 0.173, 0.435, 8, true),
      autoFactory.trajectoryCmd("D-Station")

    );
  }
}
