// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.EndAutoCommand;
import frc.robot.commands.SetActuatorL1Command;
import frc.robot.commands.SetActuatorPositionCommand;
import frc.robot.commands.SetClawPositionCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.TestClawDriveCommand;
import frc.robot.commands.TestClawDriveReverseCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
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

  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ClawSubsystem m_clawSubsystem;

  DriveCommand m_driveCommand;
  TestClawDriveCommand m_clawDriveCommand;
  TestClawDriveReverseCommand m_clawDriveReverseCommand;

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
  

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    stick = new Joystick(0);

    testOperator = new Joystick(1);
    m_swerveSubsystem = new SwerveSubsystem();
    m_elevatorSubsystem = new ElevatorSubsystem();
    m_clawSubsystem = new ClawSubsystem();
    m_driveCommand = new DriveCommand(m_swerveSubsystem, stick);
    m_clawDriveReverseCommand = new TestClawDriveReverseCommand(m_clawSubsystem);
    m_clawDriveCommand = new TestClawDriveCommand(m_clawSubsystem);

    m_setActuatorDefaultCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.DEFAULT, ClawPosition.DEFAULT);
    m_setActuatorCoralIntakeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.INTAKE, ClawPosition.INTAKE);
    m_setActuatorL1Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L1, ClawPosition.L1);
    m_setActuatorL2Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L2, ClawPosition.L2);
    m_setActuatorL3Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L3, ClawPosition.L3);
    m_setActuatorL4Command = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.L4, ClawPosition.L4);

    m_setActuatorFloorAlgaeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.FLOORALGAE, ClawPosition.FLOORALGAE);
    m_setActuatorLowerAlgaeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.LOWERALGAE, ClawPosition.REEFALGAE);
    m_setActuatorHigherAlgaeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.HIGHERALGAE, ClawPosition.REEFALGAE);
    m_setActuatorBargeCommand = new SetActuatorPositionCommand(m_elevatorSubsystem, m_clawSubsystem, ElevatorPosition.BARGE, ClawPosition.BARGE);
    



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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // JoystickButton IntakeButton = new JoystickButton(testOperator, 1);
    // IntakeButton.toggleOnTrue(m_setClawIntakeCommand);
    JoystickButton driveReverseButton = new JoystickButton(testOperator, 5);
    driveReverseButton.whileTrue(m_clawDriveReverseCommand);
    JoystickButton defaultButton = new JoystickButton(testOperator,1);
    defaultButton.onTrue(m_setClawIntakeCommand);
    JoystickButton L1Button = new JoystickButton(testOperator, 2);
    L1Button.onTrue(m_setClawL1Command);
    JoystickButton driveButton = new JoystickButton(testOperator, 6);
    driveButton.whileTrue(m_clawDriveCommand);
    // JoystickButton L2Button = new JoystickButton(testOperator, 3);
    // L2Button.toggleOnTrue(m_setClawL2Command);
    // JoystickButton L3Button = new JoystickButton(testOperator,4);
    // L3Button.toggleOnTrue(m_setClawL3Command);
    // JoystickButton L4Button = new JoystickButton(testOperator, 5);
    // L4Button.toggleOnTrue(m_setClawL4Command);



    // JoystickButton clawIntakeButton = new JoystickButton(stick, 6);//CHANGE
    // JoystickButton clawL1Button = new JoystickButton(stick,7);//CHANGE
    // JoystickButton clawL2Button = new JoystickButton(stick, 8);//CHANGE
    // JoystickButton clawL3Button = new JoystickButton(stick, 9);//CHANGE
    // JoystickButton clawL4Button = new JoystickButton(stick,10);//CHANGE

    // clawIntakeButton.onTrue(m_setClawIntakeCommand);
    // clawL1Button.onTrue(m_setClawL1Command);
    // clawL2Button.onTrue(m_setClawL2Command);
    // clawL3Button.onTrue(m_setClawL3Command);
    // clawL4Button.onTrue(m_setClawL4Command);

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
