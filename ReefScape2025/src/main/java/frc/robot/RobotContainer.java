// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.BargeCommand;
import frc.robot.commands.CoralLevelButtonCommand;
import frc.robot.commands.CoralPlacementCommand;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.EndAutoCommand;
import frc.robot.commands.GrabAlgaeCommand;

import frc.robot.commands.ReefInteractionSequentialCommand;
import frc.robot.commands.SetActuatorPositionCommand;
import frc.robot.commands.SetClawPositionCommand;
import frc.robot.commands.TestClawDriveCommand;
import frc.robot.commands.TestClawDriveReverseCommand;

import frc.robot.commands.ToggleFloorAlgaeIntakeCommand;
import frc.robot.commands.ToggleStationIntakeCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.commands.PhotonVisionCommand;
import frc.robot.commands.ProcessorCommand;
import frc.robot.commands.ReefAlgaeGrabButton;
import frc.robot.commands.ReefCoralPlacementButton;
import frc.robot.subsystems.PhotonVisionSubsystem;

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


  ToggleStationIntakeCommand m_toggleStationIntakeCommand;
  ToggleFloorAlgaeIntakeCommand m_toggleFloorAlgaeIntakeCommand;

  GrabAlgaeCommand m_grabHigherAlgaeCommand;


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
  private final PhotonVisionSubsystem m_PhotonVisionSubsytem;
  private PhotonVisionCommand m_PhotonVisionCommand;
  JoystickButton photonVisionAlignButton;

  ReefInteractionSequentialCommand m_testReef;

  ReefCoralPlacementButton m_reefCoralPlacementCommand;
  ReefAlgaeGrabButton m_ReefAlgaeGrabCommand;
  ProcessorCommand m_processorCommand;
  BargeCommand m_bargeCommand;
  

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


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoPath = new PathPlannerAuto("TestAuto");

    m_PhotonVisionSubsytem = new PhotonVisionSubsystem(m_swerveSubsystem);
    m_PhotonVisionCommand = new PhotonVisionCommand(m_PhotonVisionSubsytem, m_swerveSubsystem);

    m_reefCoralPlacementCommand = new ReefCoralPlacementButton(m_clawSubsystem);
    m_ReefAlgaeGrabCommand = new ReefAlgaeGrabButton(m_clawSubsystem);
    m_processorCommand = new ProcessorCommand(m_elevatorSubsystem, m_clawSubsystem);
    m_bargeCommand = new BargeCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem);

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
    JoystickButton L1Button = new JoystickButton(A1,1);
    L1Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 1));
    JoystickButton L2Button = new JoystickButton(A1,2);
    L2Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 2));
    JoystickButton L3Button = new JoystickButton(A1,3);
    L3Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 3));
    JoystickButton L4Button = new JoystickButton(A1,4);
    L4Button.onTrue(new CoralLevelButtonCommand(m_elevatorSubsystem, m_clawSubsystem, 4));

    //REEF TYPE BUTTON
    JoystickButton reefCoralPlacementButton = new JoystickButton(A1, 5);
    reefCoralPlacementButton.onTrue(m_reefCoralPlacementCommand);
    JoystickButton reefAlgaeGrabButton = new JoystickButton(A1,6);
    reefAlgaeGrabButton.onTrue(m_ReefAlgaeGrabCommand);
    //REEF BUTTONS
    if (m_swerveSubsystem.getIsRedAlliance()){
      JoystickButton reefAButton = new JoystickButton(A2,1);
      reefAButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 7));
      JoystickButton reefBButton = new JoystickButton(A2, 2);
      reefBButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 7));
      JoystickButton reefCButton = new JoystickButton(A2,3);
      reefCButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 8));
      JoystickButton reefDButton = new JoystickButton(A2,4);
      reefDButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 8));
      JoystickButton reefEButton = new JoystickButton(A2,5);
      reefEButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 9));
      JoystickButton reefFButton = new JoystickButton(A2,6);
      reefFButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 9));
      JoystickButton reefGButton = new JoystickButton(A2,7);
      reefGButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 10));
      JoystickButton reefHButton = new JoystickButton(A2,8);
      reefHButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 10));
      JoystickButton reefIButton = new JoystickButton(A2,9);
      reefIButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 11));
      JoystickButton reefJButton = new JoystickButton(A2,10);
      reefJButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41,11));
      JoystickButton reefKButton = new JoystickButton(A2,11);
      reefKButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 12));
      JoystickButton reefLButton = new JoystickButton(A2,12);
      reefLButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 12));
    }
    else{
      JoystickButton reefAButton = new JoystickButton(A2,1);
      reefAButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, -0.14, 0.41, 18));
      JoystickButton reefBButton = new JoystickButton(A2, 2);
      reefBButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 18));
      JoystickButton reefCButton = new JoystickButton(A2,3);
      reefCButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 17));
      JoystickButton reefDButton = new JoystickButton(A2,4);
      reefDButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 17));
      JoystickButton reefEButton = new JoystickButton(A2,5);
      reefEButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 22));
      JoystickButton reefFButton = new JoystickButton(A2,6);
      reefFButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 22));
      JoystickButton reefGButton = new JoystickButton(A2,7);
      reefGButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 21));
      JoystickButton reefHButton = new JoystickButton(A2,8);
      reefHButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 21));
      JoystickButton reefIButton = new JoystickButton(A2,9);
      reefIButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 20));
      JoystickButton reefJButton = new JoystickButton(A2,10);
      reefJButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 20));
      JoystickButton reefKButton = new JoystickButton(A2,11);
      reefKButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,-0.14,0.41, 19));
      JoystickButton reefLButton = new JoystickButton(A2,12);
      reefLButton.onTrue(new ReefInteractionSequentialCommand(m_swerveSubsystem,m_elevatorSubsystem,m_clawSubsystem, m_PhotonVisionSubsytem,+0.14,0.41, 19));
    }
    
    //ACTION BUTTONS
    JoystickButton toggleIntakeButton = new JoystickButton(A1, 7);
    toggleIntakeButton.toggleOnTrue(m_toggleStationIntakeCommand);
    JoystickButton toggleFloorAlgaeIntakeButton = new JoystickButton(A1,8);
    toggleFloorAlgaeIntakeButton.toggleOnTrue(m_toggleFloorAlgaeIntakeCommand);
    JoystickButton bargeButton = new JoystickButton(A1, 12);
    bargeButton.onTrue(m_bargeCommand);
    JoystickButton processorButton = new JoystickButton(A1, 10);
    processorButton.onTrue(m_processorCommand);

    JoystickButton driveReverseButton = new JoystickButton(testOperator, 5);
    driveReverseButton.whileTrue(m_clawDriveReverseCommand);
    JoystickButton driveButton = new JoystickButton(testOperator, 6);
    driveButton.whileTrue(m_clawDriveCommand);
    JoystickButton defaultButton = new JoystickButton(testOperator,8);
    defaultButton.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.DEFAULT, ClawPosition.DEFAULT));
    JoystickButton testL1Button = new JoystickButton(testOperator, 1);
    testL1Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L1, ClawPosition.L1));
    JoystickButton testL2Button = new JoystickButton(testOperator,2);
    testL2Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L2, ClawPosition.L2));
    JoystickButton testL3Button = new JoystickButton(testOperator,3);
    testL3Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L3, ClawPosition.L3));
    JoystickButton testL4Button = new JoystickButton(testOperator,4);
    testL4Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L4, ClawPosition.L4));
    JoystickButton intakeButton = new JoystickButton(testOperator,7);
    intakeButton.toggleOnTrue(m_toggleStationIntakeCommand);//new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.INTAKE, ClawPosition.INTAKE));
    /*JoystickButton intakeDriveButton = new JoystickButton(testOperator,7);
    intakeDriveButton.whileTrue(testIntakeCommand);*/


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    photonVisionAlignButton = new JoystickButton(stick, 6);
    photonVisionAlignButton.onTrue(new AlignCommand(m_swerveSubsystem, m_PhotonVisionSubsytem, true, 0, 1, 20));

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
