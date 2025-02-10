// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.commands.DriveCommand;
import frc.robot.commands.EndAutoCommand;
import frc.robot.commands.SetClawIntakeCommand;
import frc.robot.commands.SetClawL1Command;
import frc.robot.commands.SetClawL2Command;
import frc.robot.commands.SetClawL3Command;
import frc.robot.commands.SetClawL4Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  SwerveSubsystem m_swerveSubsystem;
  ClawSubsystem m_clawSubsystem;

  DriveCommand m_driveCommand;
  SetClawIntakeCommand m_setClawIntakeCommand;
  SetClawL1Command m_setClawL1Command;
  SetClawL2Command m_setClawL2Command;
  SetClawL3Command m_setClawL3Command;
  SetClawL4Command m_setClawL4Command;


  SendableChooser<Command> autoChooser;
  PathPlannerAuto autoPath;

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    stick = new Joystick(0);

    m_swerveSubsystem = new SwerveSubsystem();
    m_clawSubsystem = new ClawSubsystem();

    m_driveCommand = new DriveCommand(m_swerveSubsystem, stick);
    m_setClawIntakeCommand = new SetClawIntakeCommand(m_clawSubsystem);
    m_setClawL1Command = new SetClawL1Command(m_clawSubsystem);
    m_setClawL2Command = new SetClawL2Command(m_clawSubsystem);
    m_setClawL3Command = new SetClawL3Command(m_clawSubsystem);
    m_setClawL4Command = new SetClawL4Command(m_clawSubsystem);

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
    JoystickButton clawIntakeButton = new JoystickButton(stick, 0);//CHANGE
    JoystickButton clawL1Button = new JoystickButton(stick,1);//CHANGE
    JoystickButton clawL2Button = new JoystickButton(stick, 2);//CHANGE
    JoystickButton clawL3Button = new JoystickButton(stick, 3);//CHANGE
    JoystickButton clawL4Button = new JoystickButton(stick,4);//CHANGE

    clawIntakeButton.onTrue(m_setClawIntakeCommand);
    clawL1Button.onTrue(m_setClawL1Command);
    clawL2Button.onTrue(m_setClawL2Command);
    clawL3Button.onTrue(m_setClawL3Command);
    clawL4Button.onTrue(m_setClawL4Command);
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
