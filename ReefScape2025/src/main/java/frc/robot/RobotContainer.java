// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.servohub.ServoHub;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.commands.Algae3Auto;
import frc.robot.commands.AlgaeGAuto;
import frc.robot.commands.AlgaeHAuto;
import frc.robot.commands.AutoBargeCommand;
import frc.robot.commands.AutoCoralEjectCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoIntakeStopCommand;
import frc.robot.commands.AutoL4Command;
import frc.robot.commands.AutoPrepBargeCommand;
import frc.robot.commands.AutoSetStationIntakeCommand;
import frc.robot.commands.AutoWaitL4Command;
import frc.robot.commands.BargeCommand;
import frc.robot.commands.CoralLevelButtonCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.EndAutoCommand;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.commands.LeftCoralAuto;
import frc.robot.commands.ProcessorCommand;
import frc.robot.commands.ReefAlgaeGrabButton;
import frc.robot.commands.ReefCoralPlacementButton;
import frc.robot.commands.ReefInteractionSequentialCommand;
import frc.robot.commands.ReefInteractionSequentialHolderCommand;
import frc.robot.commands.ResetClawCommand;
import frc.robot.commands.ResetElevatorConfigCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetServoCommand;
import frc.robot.commands.RightCoralAuto;
import frc.robot.commands.SetActuatorPositionCommand;
import frc.robot.commands.SetClawPositionCommand;
import frc.robot.commands.StealReefAlgaeCommand;
import frc.robot.commands.TestAlgaeAlignCommand;
import frc.robot.commands.TestClawDriveCommand;
import frc.robot.commands.TestClawDriveReverseCommand;
import frc.robot.commands.TestClimbDriveMotor;
import frc.robot.commands.TestClimbDriveMotorReverse;
import frc.robot.commands.TestServoCommand;
import frc.robot.commands.TestServoIntakeCommand;
import frc.robot.commands.ToggleClimbCommand;
import frc.robot.commands.ToggleAutoFloorAlgaeIntakeCommand;
import frc.robot.commands.ToggleStationIntakeCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  ServoHub servoHub;
  AutoFactory autoFactory;

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
  ToggleAutoFloorAlgaeIntakeCommand m_toggleFloorAlgaeIntakeCommand;

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
  private final PhotonVisionSubsystem m_photonVisionSubsystem;
  JoystickButton photonVisionAlignButton;

  ReefInteractionSequentialCommand m_testReef;

  ReefCoralPlacementButton m_reefCoralPlacementCommand;
  ReefAlgaeGrabButton m_ReefAlgaeGrabCommand;
  ProcessorCommand m_processorCommand;
  BargeCommand m_bargeCommand;
  ToggleClimbCommand m_toggleClimbCommand;
  ResetServoCommand m_ResetServoCommand;

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

  ReefInteractionSequentialHolderCommand m_autoAReefCommand;
  ReefInteractionSequentialHolderCommand m_autoBReefCommand;
  ReefInteractionSequentialHolderCommand m_autoCReefCommand;
  ReefInteractionSequentialHolderCommand m_autoDReefCommand;
  ReefInteractionSequentialHolderCommand m_autoEReefCommand;
  ReefInteractionSequentialHolderCommand m_autoFReefCommand;
  ReefInteractionSequentialHolderCommand m_autoGReefCommand;
  ReefInteractionSequentialHolderCommand m_autoHReefCommand;
  ReefInteractionSequentialHolderCommand m_autoIReefCommand;
  ReefInteractionSequentialHolderCommand m_autoJReefCommand;
  ReefInteractionSequentialHolderCommand m_autoKReefCommand;
  ReefInteractionSequentialHolderCommand m_autoLReefCommand;

  StealReefAlgaeCommand m_id21StealAlgaeCommand;
  StealReefAlgaeCommand m_id20StealAlgaeCommand;
  StealReefAlgaeCommand m_id19StealAlgaeCommand;
  StealReefAlgaeCommand m_id18StealAlgaeCommand;
  StealReefAlgaeCommand m_id17StealAlgaeCommand;
  StealReefAlgaeCommand m_id22StealAlgaeCommand;
  

  ResetGyroCommand m_resetGyroCommand;
  AutoSetStationIntakeCommand m_autoSetStationIntakeCommand;
  RobotConfig config;
  ResetClawCommand m_resetClawCommand;
  
  TestServoCommand m_testServoCommand;
  TestClimbDriveMotor m_testClimbDriveMotor;
  TestClimbDriveMotorReverse m_TestClimbDriveMotorReverse;
  TestServoIntakeCommand m_TestServoIntakeCommand;
  AutoL4Command m_autoL4Command;
  AutoCoralEjectCommand m_autoCoralEjectCommand;
  AutoWaitL4Command m_autoWaitL4Command;
  AutoWaitL4Command m_autoWaitL4Command2;
  AutoIntakeCommand m_autoIntakeCommand;
  //Command testAuto;
  //Command algaeAuto;
  ResetElevatorConfigCommand resetElevatorCommand;
  ResetElevatorConfigCommand resetElevatorCommand2;
  AutoPrepBargeCommand autoPrepBargeCommand;
  AutoBargeCommand autoBargeCommand;
  AlgaeHAuto algaeHAuto;
  AlgaeGAuto algaeGAuto;
  Algae3Auto algae3Auto;
  LeftCoralAuto leftCoralAuto;
  RightCoralAuto rightCoralAuto;
  TestAlgaeAlignCommand testAlgaeAlign;
  SendableChooser<Command> m_chooser;

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
    m_climbSubsystem = new ClimbSubsystem(m_swerveSubsystem, servoHub);
    m_photonVisionSubsystem = new PhotonVisionSubsystem(m_swerveSubsystem);

    autoFactory = new AutoFactory(
            m_swerveSubsystem::getPose, // A function that returns the current robot pose
            m_swerveSubsystem::resetPose, // A function that resets the current robot pose to the provided Pose2d
            m_swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            m_swerveSubsystem // The drive subsystem
        );

    m_driveCommand = new DriveCommand(m_swerveSubsystem, stick);
    m_clawDriveReverseCommand = new TestClawDriveReverseCommand(m_clawSubsystem);
    m_clawDriveCommand = new TestClawDriveCommand(m_clawSubsystem);
    m_ResetServoCommand = new ResetServoCommand(m_intakeSubsystem);

    m_toggleStationIntakeCommand = new ToggleStationIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem);
    m_toggleFloorAlgaeIntakeCommand = new ToggleAutoFloorAlgaeIntakeCommand(m_elevatorSubsystem, m_clawSubsystem,m_photonVisionSubsystem);
    m_toggleClimbCommand = new ToggleClimbCommand(m_elevatorSubsystem, m_intakeSubsystem, m_climbSubsystem);

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

    


    m_reefCoralPlacementCommand = new ReefCoralPlacementButton(m_clawSubsystem);
    m_ReefAlgaeGrabCommand = new ReefAlgaeGrabButton(m_clawSubsystem);
    m_processorCommand = new ProcessorCommand(m_elevatorSubsystem, m_clawSubsystem);
    m_bargeCommand = new BargeCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem);
    m_resetGyroCommand = new ResetGyroCommand(m_swerveSubsystem, m_photonVisionSubsystem);
    m_setL1Level = new ReefCoralPlacementButton(m_clawSubsystem);
    m_setL2Level = new ReefCoralPlacementButton(m_clawSubsystem);
    m_setL3Level = new ReefCoralPlacementButton(m_clawSubsystem);
    m_setL4Level = new ReefCoralPlacementButton(m_clawSubsystem);

    m_reefACommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,-0.162,0.435, 7,false);
    m_reefBCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,0.173,0.435, 7,false); 
    m_reefCCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,-0.162,0.435, 8,false);
    m_reefDCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,0.173,0.435, 8,false);
    m_reefECommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,-0.162,0.435, 9,false);
    m_reefFCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,0.173,0.435, 9,false);
    m_reefGCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,-0.162,0.435, 10,false);
    m_reefHCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,0.173,0.435, 10,false);
    m_reefICommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,-0.162,0.435, 11,false); 
    m_reefJCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,0.173,0.435, 11,false);
    m_reefKCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,-0.162,0.435, 6,false);
    m_reefLCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem,0.173,0.435, 6,false);

    //AUTOPATHS
    m_autoAReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 7, true);
    m_autoBReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 0.173, 0.435, 7, true);
    m_autoCReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 8, true);
    m_autoDReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 0.173, 0.435, 8, true);
    m_autoEReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 9, true);
    m_autoFReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 0.173, 0.435, 9, true);
    m_autoGReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 10, true);
    m_autoHReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 0.173, 0.435, 10, true);
    m_autoIReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 11, true);
    m_autoJReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 0.173, 0.435, 11, true);
    m_autoKReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 6, true);
    m_autoLReefCommand = new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 0.173, 0.435, 6, true);

    m_id21StealAlgaeCommand = new StealReefAlgaeCommand(m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 21);
    m_id20StealAlgaeCommand = new StealReefAlgaeCommand(m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 20);
    m_id19StealAlgaeCommand = new StealReefAlgaeCommand(m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 19);
    m_id18StealAlgaeCommand = new StealReefAlgaeCommand(m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 18);
    m_id17StealAlgaeCommand = new StealReefAlgaeCommand(m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 17);
    m_id22StealAlgaeCommand = new StealReefAlgaeCommand(m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 22);

    m_autoSetStationIntakeCommand = new AutoSetStationIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem);
    m_autoL4Command = new AutoL4Command(m_elevatorSubsystem, m_clawSubsystem);
    m_autoCoralEjectCommand = new AutoCoralEjectCommand(m_elevatorSubsystem, m_clawSubsystem);
    m_autoWaitL4Command = new AutoWaitL4Command(m_elevatorSubsystem, m_clawSubsystem, 1.42);
    m_autoWaitL4Command2 = new AutoWaitL4Command(m_elevatorSubsystem, m_clawSubsystem, 1);
    m_autoIntakeCommand = new AutoIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem);
    
    m_testServoCommand = new TestServoCommand(m_intakeSubsystem);
    m_TestServoIntakeCommand = new TestServoIntakeCommand(m_intakeSubsystem);
    resetElevatorCommand = new ResetElevatorConfigCommand(m_elevatorSubsystem);
    resetElevatorCommand2 = new ResetElevatorConfigCommand(m_elevatorSubsystem);
    autoPrepBargeCommand = new AutoPrepBargeCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem);
    autoBargeCommand = new AutoBargeCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem);


    algaeHAuto = new AlgaeHAuto(autoFactory,m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem);
    algaeGAuto = new AlgaeGAuto(autoFactory, m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem);
    leftCoralAuto = new LeftCoralAuto(autoFactory, m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, m_intakeSubsystem);
    rightCoralAuto = new RightCoralAuto(autoFactory, m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, m_intakeSubsystem);


    testAlgaeAlign = new TestAlgaeAlignCommand(m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem);
    //algae3Auto = new Algae3Auto(autoFactory, m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_autoHReefCommand, m_autoIReefCommand, m_autoFReefCommand);
    // NamedCommands.registerCommand("m_autoIntakeCommand",m_autoIntakeCommand);
    // NamedCommands.registerCommand("m_ReefAlgaeGrabCommand",m_ReefAlgaeGrabCommand);
    // NamedCommands.registerCommand("m_reefCoralPlacementCommand",m_reefCoralPlacementCommand);
    // NamedCommands.registerCommand("m_SetL4Level", m_setL4Level);
    // NamedCommands.registerCommand("m_autoAReefCommand",m_autoAReefCommand);
    // NamedCommands.registerCommand("m_autoBReefCommand",m_autoBReefCommand);
    // NamedCommands.registerCommand("m_autoCReefCommand",m_autoCReefCommand);
    // NamedCommands.registerCommand("m_autoDReefCommand",m_autoDReefCommand);
    // NamedCommands.registerCommand("m_autoEReefCommand",m_autoEReefCommand);
    // NamedCommands.registerCommand("m_autoFReefCommand",m_autoFReefCommand);
    // NamedCommands.registerCommand("m_autoGReefCommand",m_autoGReefCommand);
    // NamedCommands.registerCommand("m_autoHReefCommand",m_autoHReefCommand);
    // NamedCommands.registerCommand("m_autoIReefCommand",m_autoIReefCommand);
    // NamedCommands.registerCommand("m_autoJReefCommand",m_autoJReefCommand);
    // NamedCommands.registerCommand("m_autoKReefCommand",m_autoKReefCommand);
    // NamedCommands.registerCommand("m_autoLReefCommand",m_autoLReefCommand);
    
    // NamedCommands.registerCommand("m_toggleStationIntakeCommand", m_toggleStationIntakeCommand);
    // NamedCommands.registerCommand("m_autoSetStationIntakeCommand", m_autoSetStationIntakeCommand);
    // NamedCommands.registerCommand("m_autoL4Command", m_autoL4Command);
    // NamedCommands.registerCommand("m_autoCoralEjectCommand", m_autoCoralEjectCommand);
    // NamedCommands.registerCommand("m_autoWaitL4Command", m_autoWaitL4Command);
    // NamedCommands.registerCommand("m_autoWaitL4Command2", m_autoWaitL4Command2);

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    m_resetClawCommand = new ResetClawCommand(m_clawSubsystem,A1);
    m_testClimbDriveMotor = new TestClimbDriveMotor(m_climbSubsystem);
    m_TestClimbDriveMotorReverse = new TestClimbDriveMotorReverse(m_climbSubsystem);
    //test;
    //autoPath.andThen(new EndAutoCommand(m_swerveSubsystem));
    
    // testAuto = Commands.sequence(
    //   new ReefAlgaeGrabButton(m_clawSubsystem),
    //   autoFactory.resetOdometry("Start-Eoffset"),
    //   autoFactory.trajectoryCmd("Start-Eoffset"),
    //   new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 9, true),

    //   Commands.deadline(
    //     new AutoIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem),
    //     autoFactory.trajectoryCmd("E-Station").andThen(new AutoIntakeStopCommand(m_swerveSubsystem))
    //   ),

    //   new ResetElevatorConfigCommand(m_elevatorSubsystem),
    //   autoFactory.trajectoryCmd("Station-Coffset"),
    //   new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, -0.162, 0.435, 8, true),

    //   Commands.deadline(
    //     new AutoIntakeCommand(m_elevatorSubsystem, m_clawSubsystem, m_intakeSubsystem),
    //     autoFactory.trajectoryCmd("C-Station").andThen(new AutoIntakeStopCommand(m_swerveSubsystem))
    //   ),
      
    //   new ResetElevatorConfigCommand(m_elevatorSubsystem),
    //   autoFactory.trajectoryCmd("Station-Doffset"),
    //   new ReefInteractionSequentialHolderCommand(m_swerveSubsystem, m_elevatorSubsystem, m_clawSubsystem, m_photonVisionSubsystem, 0.173, 0.435, 8, true),
    //   autoFactory.trajectoryCmd("D-Station")
    // );
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Mid H Algae Auto", algaeHAuto);
    m_chooser.addOption("Mid G Algae Auto", algaeGAuto);
    m_chooser.addOption("Left Coral Auto", leftCoralAuto);
    m_chooser.addOption("Right Coral Auto", rightCoralAuto);
    
    // algaeAuto = Commands.sequence(
    //   autoFactory.resetOdometry("Start-Hoffset"),
    //   autoFactory.trajectoryCmd("Start-Hoffset"),
    //   m_autoHReefCommand,
    //   autoFactory.trajectoryCmd("Algae21-Barge1"),
    //   autoPrepBargeCommand,
    //   autoBargeCommand,
    //   autoFactory.trajectoryCmd("Barge1-Algae20"),
    //   resetElevatorCommand,
    //   m_reefCoralPlacementCommand,
    //   m_autoIReefCommand,
    //   autoFactory.trajectoryCmd("Algae20-Barge1"),
    //   autoPrepBargeCommand,
    //   autoBargeCommand,
    //   autoFactory.trajectoryCmd("Barge1-Algae22")
    // );
    configureBindings();
  }
  public SwerveSubsystem getSwerveSubsystem(){
    return m_swerveSubsystem;
  }
  // public Command testAuto(){
  //   return Commands.sequence(
  //     autoFactory.resetOdometry("Start-E"),
  //     autoFactory.trajectoryCmd("Start-E"),
  //     autoFactory.trajectoryCmd("E-Station"),
  //     autoFactory.trajectoryCmd("Station-C")
  //     //autoFactory.trajectoryCmd("E-Station")
  //   );
  // }
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

    JoystickButton stealAlgae21LeftButton = new JoystickButton(A2, 17);
    stealAlgae21LeftButton.toggleOnTrue(m_id21StealAlgaeCommand);
    JoystickButton stealAlgae21RightButton = new JoystickButton(A2, 18);
    stealAlgae21RightButton.toggleOnTrue(m_id21StealAlgaeCommand);
    JoystickButton stealAlgae20LeftButton = new JoystickButton(A2, 19);
    stealAlgae20LeftButton.toggleOnTrue(m_id20StealAlgaeCommand);
    JoystickButton stealAlgae20RightButton = new JoystickButton(A2, 20);
    stealAlgae20RightButton.toggleOnTrue(m_id20StealAlgaeCommand);
    JoystickButton stealAlgae19LeftButton = new JoystickButton(A2, 21);
    stealAlgae19LeftButton.toggleOnTrue(m_id19StealAlgaeCommand);
    JoystickButton stealAlgae19RightButton = new JoystickButton(A2, 22);
    stealAlgae19RightButton.toggleOnTrue(m_id19StealAlgaeCommand);
    JoystickButton stealAlgae18LeftButton = new JoystickButton(A2, 23);
    stealAlgae18LeftButton.toggleOnTrue(m_id18StealAlgaeCommand);
    JoystickButton stealAlgae18RightButton = new JoystickButton(A2, 24);
    stealAlgae18RightButton.toggleOnTrue(m_id18StealAlgaeCommand);
    JoystickButton stealAlgae17LeftButton = new JoystickButton(A2, 25);
    stealAlgae17LeftButton.toggleOnTrue(m_id17StealAlgaeCommand);
    JoystickButton stealAlgae17RightButton = new JoystickButton(A2, 26);
    stealAlgae17RightButton.toggleOnTrue(m_id17StealAlgaeCommand);
    JoystickButton stealAlgae22LeftButton = new JoystickButton(A2, 27);
    stealAlgae22LeftButton.toggleOnTrue(m_id22StealAlgaeCommand);
    JoystickButton stealAlgae22RightButton = new JoystickButton(A2,28);
    stealAlgae22RightButton.toggleOnTrue(m_id22StealAlgaeCommand);

    //ACTION BUTTONS
    JoystickButton toggleIntakeButton = new JoystickButton(A1, 7);
    toggleIntakeButton.toggleOnTrue(m_toggleStationIntakeCommand);
    JoystickButton toggleFloorAlgaeIntakeButton = new JoystickButton(A1,8);
    toggleFloorAlgaeIntakeButton.toggleOnTrue(m_toggleFloorAlgaeIntakeCommand);
    JoystickButton bargeButton = new JoystickButton(A1, 12);
    bargeButton.onTrue(m_bargeCommand);
    JoystickButton processorButton = new JoystickButton(A1, 11);
    processorButton.onTrue(m_processorCommand);
    JoystickButton climbButton = new JoystickButton(A1, 10);
    climbButton.onTrue(m_toggleClimbCommand);
    //JoystickButton wristResetButton = new JoystickButton(A1,11);
    //wristResetButton.whileTrue(m_resetClawCommand);
    //DRIVER BUTTONS-
    JoystickButton resetGyroButton = new JoystickButton(stick, 7);
    resetGyroButton.onTrue(m_resetGyroCommand);

    JoystickButton driveReverseButton = new JoystickButton(testOperator, 5);
    driveReverseButton.whileTrue(m_clawDriveCommand);
    JoystickButton driveButton = new JoystickButton(testOperator, 6);
    driveButton.whileTrue(m_clawDriveReverseCommand);
    JoystickButton defaultButton = new JoystickButton(testOperator,8);
    defaultButton.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.DEFAULT, ClawPosition.DEFAULT));
    JoystickButton testL1Button = new JoystickButton(testOperator, 1);
    testL1Button.onTrue(new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L1, ClawPosition.L1));
    // JoystickButton testAlgaeAlignButton = new JoystickButton(testOperator,1);
    // testAlgaeAlignButton.toggleOnTrue(testAlgaeAlign);
    // JoystickButton testIntakeServo = new JoystickButton(testOperator, 1);
    // testIntakeServo.toggleOnTrue(m_testServoCommand);

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
    //JoystickButton testClimbForwardButton = new JoystickButton(testOperator, 11);
    // testClimbForwardButton.whileTrue(testClimbDriveMotor);
    // JoystickButton testClimbReverseButton = new JoystickButton(testOperator,12);
    // testClimbReverseButton.whileTrue(testClimbDriveMotorReverse);
    // JoystickButton testServoButton = new JoystickButton(testOperator,7);
    // testServoButton.toggleOnTrue(m_testServoCommand);
    // JoystickButton testServoIntakeButton = new JoystickButton (testOperator, 8);
    // testServoIntakeButton.toggleOnTrue(m_TestServoIntakeCommand);
    //JoystickButton testClimbDriveMotor = new JoystickButton(testOperator, 7);
    //testClimbDriveMotor.whileTrue(m_testClimbDriveMotor);
    //JoystickButton intakeDriveButton = new JoystickButton(testOperator,7);
    //intakeDriveButton.whileTrue(testIntakeCommand);

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //photonVisionAlignButton = new JoystickButton(stick, 6);
    //photonVisionAlignButton.onTrue(new AlignCommand(m_swerveSubsystem, m_photonVisionSubsystem, true, 0, 1, 20));
    SmartDashboard.putData(m_chooser);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getTeleopCommand(){
    return m_driveCommand;
  }
  public Command getResetIntakeCommand(){
    return m_ResetServoCommand;
  }
  /*public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }*/
  public Command getAutonomousCommand(){
    //return new PathPlannerAuto("New Auto");
    //return autoPath.andThen(new EndAutoCommand(m_swerveSubsystem));
    //return autoChooser.getSelected().andThen(new EndAutoCommand(m_swerveSubsystem));
    return m_chooser.getSelected().andThen(new EndAutoCommand(m_swerveSubsystem));
  }
  public PhotonVisionSubsystem getPhotonSubsystem(){
    return m_photonVisionSubsystem;
  }
  public ClawSubsystem getClawSubsystem(){
    return m_clawSubsystem;
  }
  public ResetClawCommand getResetClawCommand(){
    return m_resetClawCommand;
  }
}
