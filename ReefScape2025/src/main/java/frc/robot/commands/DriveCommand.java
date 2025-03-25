// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vector;

public class DriveCommand extends Command {
  //SWERVE SUBSYSTEM FROM ROBOT CONTAINER
  SwerveSubsystem m_swerveSubsystem;
  //JOYSTICK FROM ROBOT CONTAINER
  Joystick m_driverJoystick;
  //DRIVING VECTORS
  Vector strafeVector;
  Vector rotationVector;
  ChassisSpeeds chassisSpeed;

  public DriveCommand(SwerveSubsystem swerveSubsystem, Joystick stick) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_driverJoystick = stick;

    //strafeVector = new Vector(0,0);
    //rotationVector = new Vector(0,0);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() { 
    //JOYSTICK VALUES FROM LEFT JOYSTICK
    strafeVector = new Vector(m_driverJoystick.getRawAxis(0),-m_driverJoystick.getRawAxis(1));
    //JOYSTICK VALUES FROM THE RIGHT JOYSTICK
    rotationVector = new Vector(m_driverJoystick.getRawAxis(4),-m_driverJoystick.getRawAxis(5));
    //rotationVector = new Vector(1,(rotationVector.getDegrees() + 180) % 360,true);
    //chassisSpeed = new ChassisSpeeds(0, 1,0);
    chassisSpeed = new ChassisSpeeds(strafeVector.getX() * 2,strafeVector.getY() * 2,-rotationVector.getX() * 3);
    //m_swerveSubsystem.velocityControlledDrive(chassisSpeed);
    //strafeVector = new Vector(0, 0);
    //rotationVector = new Vector(0, 0);
    //System.out.println("hello");
    //strafeVector = new Vector(0, 0.2);
    //rotationVector = new Vector(0,0);
    m_swerveSubsystem.driverControlledDrive(strafeVector,rotationVector);
    //m_swerveSubsystem.velocityControlledDrive(chassisSpeed);
  }
  
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
