// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.commands.CoralLevelButtonCommand;
import frc.robot.commands.CoralPlacementSequenceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.EndAutoCommand;
import frc.robot.commands.ReefInteractionSequentialCommand;
import frc.robot.commands.SetActuatorPositionCommand;
import frc.robot.commands.SetClawPositionCommand;
import frc.robot.commands.TestClawDriveCommand;
import frc.robot.commands.TestClawDriveReverseCommand;
import frc.robot.commands.TestIntakeCommand;
import frc.robot.commands.ToggleFloorAlgaeIntakeCommand;
import frc.robot.commands.ToggleStationIntakeCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.lang.annotation.ElementType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  Joystick stick;
  Joystick testOperator;
  Joystick A1;
  Joystick A2;

  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  DriveCommand m_driveCommand;
  TestClawDriveCommand m_clawDriveCommand;
  TestClawDriveReverseCommand m_clawDriveReverseCommand;
  TestIntakeCommand testIntakeCommand;

  ToggleStationIntakeCommand m_toggleStationIntakeCommand;
  ToggleFloorAlgaeIntakeCommand m_toggleFloorAlgaeIntakeCommand;

  SetClawPositionCommand testClaw1Command;
  SetClawPositionCommand testClaw2Command;
  SetClawPositionCommand testClaw3Command;
  SetClawPositionCommand testClaw4Command;

  SetActuatorPositionCommand m_setActuatorDefaultCommand;

  SetActuatorPositionCommand m_setActuatorCoralIntakeCommand;
  SetActuatorPositionCommand m_setActuatorL1Command;
  SetActuatorPositionCommand m_setActuatorL2Command;
  SetActuatorPositionCommand m_setActuatorL3Command;
  SetActuatorPositionCommand m_setActuatorL4Command;

  SetActuatorPositionCommand m_setActuatorFloorAlgaeCommand;
  SetActuatorPositionCommand m_setActuatorLowerAlgaeCommand;
  SetActuatorPositionCommand m_setActuatorHigherAlgaeCommand;
  SetActuatorPositionCommand m_setActuatorBargeCommand;
  
  SendableChooser<Command> autoChooser;
  PathPlannerAuto autoPath;

  ReefInteractionSequentialCommand m_testReef;
  

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    A1 = new Joystick(0);
    A2 = new Joystick(1);
    stick = new Joystick(2);

    testOperator = new Joystick(3);
    m_swerveSubsystem = new SwerveSubsystem();
    m_elevatorSubsystem = new ElevatorSubsystem(m_swerveSubsystem);
    m_clawSubsystem = new ClawSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_driveCommand = new DriveCommand(m_swerveSubsystem, stick);
    m_clawDriveReverseCommand = new TestClawDriveReverseCommand(m_clawSubsystem);
    m_clawDriveCommand = new TestClawDriveCommand(m_clawSubsystem);

    m_toggleStationIntakeCommand = new ToggleStationIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem);
    m_toggleFloorAlgaeIntakeCommand = new ToggleFloorAlgaeIntakeCommand(m_elevatorSubsystem, m_clawSubsystem);

    m_setActuatorDefaultCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.DEFAULT, ClawPosition.DEFAULT);
    m_setActuatorCoralIntakeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.INTAKE, ClawPosition.INTAKE);
    m_setActuatorL1Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L1, ClawPosition.L1);
    m_setActuatorL2Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L2, ClawPosition.L2);
    m_setActuatorL3Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L3, ClawPosition.L3);
    m_setActuatorL4Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L4, ClawPosition.L4);

    m_setActuatorFloorAlgaeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.FLOORALGAE, ClawPosition.FLOORALGAE);

    m_setActuatorBargeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.BARGE, ClawPosition.BARGE);

    testClaw1Command = new SetClawPositionCommand(m_clawSubsystem, ClawPosition.DEFAULT);
    testClaw2Command = new SetClawPositionCommand(m_clawSubsystem, ClawPosition.L2);
    testIntakeCommand = new TestIntakeCommand(m_intakeSubsystem);


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoPath = new PathPlannerAuto("TestAuto");
    
    //autoPath.andThen(new EndAutoDrive(m_swerveSubsystem));
    configureBindings();
  }
  public SwerveSubsystem getSwerveSubsystem(){
    return m_swerveSubsystem;
  }
  /*public Command getAutonomousCommand(){
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile("AutoPath1");
      return AutoBuilder.followPath(path);
    } catch(Exception e)
{
  DriverStation.reportError("Error Alert: " + e.getMessage(), e.getStackTrace());
  return Commands.none();
}  }
*/
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //LEVEL BUTTONS
    JoystickButton L1Button = new JoystickButton(A2,1);
    L1Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 1));
    JoystickButton L2Button = new JoystickButton(A2,2);
    L2Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 2));
    JoystickButton L3Button = new JoystickButton(A2,3);
    L3Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 3));
    JoystickButton L4Button = new JoystickButton(A2,4);
    L4Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 4));
    //REEF BUTTONS
    JoystickButton reef1Button = new JoystickButton(A1,1);
    reef1Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef2Button = new JoystickButton(A1, 2);
    reef2Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef3Button = new JoystickButton(A1,3);
    reef3Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef4Button = new JoystickButton(A1,4);
    reef4Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef5Button = new JoystickButton(A1,5);
    reef5Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef6Button = new JoystickButton(A1,6);
    reef6Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef7Button = new JoystickButton(A1,7);
    reef7Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef8Button = new JoystickButton(A1,8);
    reef8Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef9Button = new JoystickButton(A1,9);
    reef9Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef10Button = new JoystickButton(A1,10);
    reef10Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef11Button = new JoystickButton(A1,11);
    reef11Button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    JoystickButton reef12button = new JoystickButton(A1,12);
    reef12button.onTrue(new CoralPlacementSequenceCommand(m_elevatorSubsystem, m_clawSubsystem));
    //ACTION BUTTONS
    JoystickButton toggleIntakeButton = new JoystickButton(A2, 4);
    toggleIntakeButton.toggleOnTrue(m_toggleStationIntakeCommand);
    JoystickButton toggleFloorAlgaeIntakeButton = new JoystickButton(A2,5);
    toggleFloorAlgaeIntakeButton.toggleOnTrue(m_toggleFloorAlgaeIntakeCommand);

    JoystickButton driveReverseButton = new JoystickButton(testOperator, 5);
    driveReverseButton.whileTrue(m_clawDriveReverseCommand);
    JoystickButton driveButton = new JoystickButton(testOperator, 6);
    driveButton.whileTrue(m_clawDriveCommand);
    JoystickButton defaultButton = new JoystickButton(testOperator,10);
    defaultButton.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.DEFAULT, ClawPosition.DEFAULT));
    JoystickButton testL1Button = new JoystickButton(testOperator, 1);
    testL1Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L1, ClawPosition.L1));
    JoystickButton testL2Button = new JoystickButton(testOperator,2);
    testL2Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L2, ClawPosition.L2));
    JoystickButton testL3Button = new JoystickButton(testOperator,3);
    testL3Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L3, ClawPosition.L3));
    JoystickButton testL4Button = new JoystickButton(testOperator,4);
    testL4Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L4, ClawPosition.L4));//new ReefInteractionSequentialCommand(m_elevatorSubsystem, m_clawSubsystem,ElevatorPosition.L3,ClawPosition.L3,21));//new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L4, ClawPosition.L4));
    JoystickButton intakeButton = new JoystickButton(testOperator,8);
    intakeButton.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.INTAKE, ClawPosition.INTAKE));
    JoystickButton intakeDriveButton = new JoystickButton(testOperator,7);
    intakeDriveButton.whileTrue(testIntakeCommand);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getTeleopCommand(){
    return m_driveCommand;
  }
  /*public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }*/
  public Command getAutonomousCommand(){
    //return new PathPlannerAuto("New Auto");
    return autoPath.andThen(new EndAutoCommand(m_swerveSubsystem));
  }
}
