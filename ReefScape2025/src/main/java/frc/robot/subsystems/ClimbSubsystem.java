// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX driveMotor;
  ServoHub m_servoHub;
  PIDController climbController;
  boolean toggleClimb;
  double targetPosition;
  double climbCalculate;
  public ClimbSubsystem(ServoHub servoHub) {
    driveMotor = new TalonFX(41);//CHANGEEEEEEEEEEEEEEEEEE
    m_servoHub = servoHub;
    climbController = new PIDController(0.01, 0, 0);
    climbController.setTolerance(.001);
    toggleClimb = false;
    targetPosition = 0;
    climbCalculate = 0;
  }
  public void prepClimb(){
    targetPosition = 100;//CHANGEEEEEEEEEEEEEEEE
  }

  public void climb(){
    targetPosition = 0;
    
  }
  @Override
  public void periodic() {
    climbCalculate = climbController.calculate(driveMotor.getPosition().getValueAsDouble(), targetPosition);
    driveMotor.set(climbCalculate);
    

    
    // This method will be called once per scheduler run
  }
}
