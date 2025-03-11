// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AutoSetStationIntakeCommand;
import frc.robot.commands.BargeCommand;
import frc.robot.commands.CoralLevelButtonCommand;
import frc.robot.commands.CoralPlacementCommand;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.EndAutoCommand;
import frc.robot.commands.GrabAlgaeCommand;

import frc.robot.commands.ReefInteractionSequentialCommand;
import frc.robot.commands.ReefInteractionSequentialHolderCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.SetActuatorPositionCommand;
import frc.robot.commands.SetClawPositionCommand;
import frc.robot.commands.TestClawDriveCommand;
import frc.robot.commands.TestClawDriveReverseCommand;
import frc.robot.commands.TestClimbDriveMotor;
import frc.robot.commands.TestClimbDriveMotorReverse;
import frc.robot.commands.ToggleClimbCommand;
import frc.robot.commands.ToggleFloorAlgaeIntakeCommand;
import frc.robot.commands.ToggleStationIntakeCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


import frc.robot.commands.ProcessorCommand;
import frc.robot.commands.ReefAlgaeGrabButton;
import frc.robot.commands.ReefCoralPlacementButton;
import frc.robot.subsystems.PhotonVisionSubsystem;

import frc.robot.subsystems.SwerveSubsystem;

import java.io.BufferedWriter;
import java.lang.annotation.ElementType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  ServoHub servoHub;

  TestClimbDriveMotor testClimbDriveMotor;
  TestClimbDriveMotorReverse testClimbDriveMotorReverse;
  
  Joystick stick;
  Joystick testOperator;
  Joystick A1;
  Joystick A2;

  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ClimbSubsystem m_climbSubsystem;

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
  JoystickButton photonVisionAlignButton;

  ReefInteractionSequentialCommand m_testReef;

  ReefCoralPlacementButton m_reefCoralPlacementCommand;
  ReefAlgaeGrabButton m_ReefAlgaeGrabCommand;
  ProcessorCommand m_processorCommand;
  BargeCommand m_bargeCommand;
  ToggleClimbCommand m_toggleClimbCommand;

  ReefCoralPlacementButton m_setL1Level;
  ReefCoralPlacementButton m_setL2Level;
  ReefCoralPlacementButton m_setL3Level;
  ReefCoralPlacementButton m_setL4Level;

  ReefInteractionSequentialHolderCommand m_reefACommand;
  ReefInteractionSequentialHolderCommand m_reefBCommand;
  ReefInteractionSequentialHolderCommand m_reefCCommand;
  ReefInteractionSequentialHolderCommand m_reefDCommand;
  ReefInteractionSequentialHolderCommand m_reefECommand;
  ReefInteractionSequentialHolderCommand m_reefFCommand;
  ReefInteractionSequentialHolderCommand m_reefGCommand;
  ReefInteractionSequentialHolderCommand m_reefHCommand;
  ReefInteractionSequentialHolderCommand m_reefICommand;
  ReefInteractionSequentialHolderCommand m_reefJCommand;
  ReefInteractionSequentialHolderCommand m_reefKCommand;
  ReefInteractionSequentialHolderCommand m_reefLCommand;




  ReefInteractionSequentialCommand m_autoAReefCommand;
  ReefInteractionSequentialCommand m_autoBReefCommand;
  ReefInteractionSequentialCommand m_autoCReefCommand;
  ReefInteractionSequentialCommand m_autoDReefCommand;
  ReefInteractionSequentialCommand m_autoEReefCommand;
  ReefInteractionSequentialCommand m_autoFReefCommand;
  ReefInteractionSequentialCommand m_autoGReefCommand;
  ReefInteractionSequentialCommand m_autoHReefCommand;
  ReefInteractionSequentialCommand m_autoIReefCommand;
  ReefInteractionSequentialCommand m_autoJReefCommand;
  ReefInteractionSequentialCommand m_autoKReefCommand;
  ReefInteractionSequentialCommand m_autoLReefCommand;

  ResetGyroCommand m_resetGyroCommand;
  AutoSetStationIntakeCommand m_autoSetStationIntakeCommand;
  RobotConfig config;
  
  

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    servoHub = new ServoHub(40);////CHANGEEEEEEEEEEEEEEEEEEEEEEEEE
    A1 = new Joystick(0);
    A2 = new Joystick(1);
    stick = new Joystick(2);

    testOperator = new Joystick(3);
    m_swerveSubsystem = new SwerveSubsystem();
    m_elevatorSubsystem = new ElevatorSubsystem(m_swerveSubsystem);
    m_clawSubsystem = new ClawSubsystem();
    m_intakeSubsystem = new IntakeSubsystem(servoHub);
    m_climbSubsystem = new ClimbSubsystem(servoHub);

    m_driveCommand = new DriveCommand(m_swerveSubsystem, stick);
    m_clawDriveReverseCommand = new TestClawDriveReverseCommand(m_clawSubsystem);
    m_clawDriveCommand = new TestClawDriveCommand(m_clawSubsystem);

    m_toggleStationIntakeCommand = new ToggleStationIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem);
    m_toggleFloorAlgaeIntakeCommand = new ToggleFloorAlgaeIntakeCommand(m_elevatorSubsystem, m_clawSubsystem);
    m_toggleClimbCommand = new ToggleClimbCommand(m_intakeSubsystem, m_climbSubsystem);

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

    testClimbDriveMotor = new TestClimbDriveMotor(m_climbSubsystem);
    testClimbDriveMotorReverse = new TestClimbDriveMotorReverse(m_climbSubsystem);


    
    //autoPath = new PathPlannerAuto("TestAuto");

    m_PhotonVisionSubsytem = new PhotonVisionSubsystem(m_swerveSubsystem);


    m_reefCoralPlacementCommand = new ReefCoralPlacementButton(m_clawSubsystem);
    m_ReefAlgaeGrabCommand = new ReefAlgaeGrabButton(m_clawSubsystem);
    m_processorCommand = new ProcessorCommand(m_elevatorSubsystem, m_clawSubsystem);
    m_bargeCommand = new BargeCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem);
    m_resetGyroCommand = new ResetGyroCommand(m_swerveSubsystem, m_PhotonVisionSubsytem);
    m_setL1Level = new ReefCoralPlacementButton(m_clawSubsystem);
    m_setL2Level = new ReefCoralPlacementButton(m_clawSubsystem);
    m_setL3Level = new ReefCoralPlacementButton(m_clawSubsystem);
    m_setL4Level = new ReefCoralPlacementButton(m_clawSubsystem);

    m_reefACommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,-0.165,0.416, 7,false);
    m_reefBCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,0.165,0.416, 7,false); 
    m_reefCCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,-0.165,0.416, 8,false);
    m_reefDCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,0.165,0.416, 8,false);
    m_reefECommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,-0.165,0.416, 9,false);
    m_reefFCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,0.165,0.416, 9,false);
    m_reefGCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,-0.165,0.416, 10,false);
    m_reefHCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,0.165,0.416, 10,false);
    m_reefICommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,-0.165,0.416, 11,false); 
    m_reefJCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,0.165,0.416, 11,false);
    m_reefKCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,-0.165,0.416, 6,false);
    m_reefLCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem,0.165,0.416, 6,false);

    //AUTOPATHS
    m_autoAReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, -0.165, 0.416, 18, true);
    m_autoBReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, 0.17, 0.416, 18, true);
    m_autoCReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, -0.165, 0.416, 17, true);
    m_autoDReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, 0.17, 0.416, 17, true);//new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, 0.17, 0.416, 17, true);
    m_autoEReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, -0.165, 0.416, 22, true);
    m_autoFReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, 0.17, 0.416, 22, true);
    m_autoGReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, -0.165, 0.416, 21, true);
    m_autoHReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, 0.17, 0.416, 21, true);
    m_autoIReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, -0.165, 0.416, 20, true);
    m_autoJReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, 0.17, 0.416, 20, true);
    m_autoKReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, -0.165, 0.416, 19, true);
    m_autoLReefCommand = new ReefInteractionSequentialCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_PhotonVisionSubsytem, 0.17, 0.416, 19, true);

    m_autoSetStationIntakeCommand = new AutoSetStationIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem);
    
    
    NamedCommands.registerCommand("m_ReefAlgaeGrabCommand",m_ReefAlgaeGrabCommand);
    NamedCommands.registerCommand("m_reefCoralPlacementCommand",m_reefCoralPlacementCommand);
    NamedCommands.registerCommand("m_SetL4Level", m_setL4Level);
    NamedCommands.registerCommand("m_autoAReefCommand",m_autoAReefCommand);
    NamedCommands.registerCommand("m_autoBReefCommand",m_autoBReefCommand);
    NamedCommands.registerCommand("m_autoCReefCommand",m_autoCReefCommand);
    NamedCommands.registerCommand("m_autoDReefCommand",m_autoDReefCommand);
    NamedCommands.registerCommand("m_autoEReefCommand",m_autoEReefCommand);
    NamedCommands.registerCommand("m_autoFReefCommand",m_autoFReefCommand);
    NamedCommands.registerCommand("m_autoGReefCommand",m_autoGReefCommand);
    NamedCommands.registerCommand("m_autoHReefCommand",m_autoHReefCommand);
    NamedCommands.registerCommand("m_autoIReefCommand",m_autoIReefCommand);
    NamedCommands.registerCommand("m_autoJReefCommand",m_autoJReefCommand);
    NamedCommands.registerCommand("m_autoKReefCommand",m_autoKReefCommand);
    NamedCommands.registerCommand("m_autoLReefCommand",m_autoLReefCommand);
    
    NamedCommands.registerCommand("m_toggleStationIntakeCommand", m_toggleStationIntakeCommand);
    NamedCommands.registerCommand("m_autoSetStationIntakeCommand", m_autoSetStationIntakeCommand);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //test;
    //autoPath.andThen(new EndAutoCommand(m_swerveSubsystem));
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
    
    JoystickButton reefAButton = new JoystickButton(A2,1);
    reefAButton.toggleOnTrue(m_reefACommand);
    JoystickButton reefBButton = new JoystickButton(A2, 2);
    reefBButton.toggleOnTrue(m_reefBCommand);

    JoystickButton reefCButton = new JoystickButton(A2,3);
    reefCButton.toggleOnTrue(m_reefCCommand);
    JoystickButton reefDButton = new JoystickButton(A2,4);
    reefDButton.toggleOnTrue(m_reefDCommand);

    JoystickButton reefEButton = new JoystickButton(A2,5);
    reefEButton.toggleOnTrue(m_reefECommand);
    JoystickButton reefFButton = new JoystickButton(A2,6);
    reefFButton.toggleOnTrue(m_reefFCommand);

    JoystickButton reefGButton = new JoystickButton(A2,7);
    reefGButton.toggleOnTrue(m_reefGCommand);
    JoystickButton reefHButton = new JoystickButton(A2,8);
    reefHButton.toggleOnTrue(m_reefHCommand);

    JoystickButton reefIButton = new JoystickButton(A2,9);
    reefIButton.toggleOnTrue(m_reefICommand);
    JoystickButton reefJButton = new JoystickButton(A2,10);
    reefJButton.toggleOnTrue(m_reefJCommand);

    JoystickButton reefKButton = new JoystickButton(A2,11);
    reefKButton.toggleOnTrue(m_reefKCommand);
    JoystickButton reefLButton = new JoystickButton(A2,12);
    reefLButton.toggleOnTrue(m_reefLCommand);
    
    //ACTION BUTTONS
    JoystickButton toggleIntakeButton = new JoystickButton(A1, 7);
    toggleIntakeButton.toggleOnTrue(m_toggleStationIntakeCommand);
    JoystickButton toggleFloorAlgaeIntakeButton = new JoystickButton(A1,8);
    toggleFloorAlgaeIntakeButton.toggleOnTrue(m_toggleFloorAlgaeIntakeCommand);
    JoystickButton bargeButton = new JoystickButton(A1, 12);
    bargeButton.onTrue(m_bargeCommand);
    JoystickButton processorButton = new JoystickButton(A1, 10);
    processorButton.onTrue(m_processorCommand);
    JoystickButton climbButton = new JoystickButton(A1, 11);
    climbButton.onTrue(m_toggleClimbCommand);

    //DRIVER BUTTONS
    JoystickButton resetGyroButton = new JoystickButton(stick, 7);
    resetGyroButton.onTrue(m_resetGyroCommand);

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
    JoystickButton groundIntakeButton = new JoystickButton(testOperator, 9);
    groundIntakeButton.toggleOnTrue(m_toggleFloorAlgaeIntakeCommand);
    JoystickButton bargeButtonTest = new JoystickButton(testOperator, 10);
    bargeButtonTest.onTrue(m_bargeCommand);
    JoystickButton testClimbForwardButton = new JoystickButton(testOperator, 11);
    testClimbForwardButton.whileTrue(testClimbDriveMotor);
    JoystickButton testClimbReverseButton = new JoystickButton(testOperator,12);
    testClimbReverseButton.whileTrue(testClimbDriveMotorReverse);
    /*JoystickButton intakeDriveButton = new JoystickButton(testOperator,7);
    intakeDriveButton.whileTrue(testIntakeCommand);*/

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //photonVisionAlignButton = new JoystickButton(stick, 6);
    //photonVisionAlignButton.onTrue(new AlignCommand(m_swerveSubsystem, m_PhotonVisionSubsytem, true, 0, 1, 20));
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
    //return autoPath.andThen(new EndAutoCommand(m_swerveSubsystem));
    return autoChooser.getSelected().andThen(new EndAutoCommand(m_swerveSubsystem));
  }
  public PhotonVisionSubsystem getPhotonSubsystem(){
    return m_PhotonVisionSubsytem;
  }
}
