// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX driveMotor;
  ServoHub m_servoHub;
  PIDController climbController;
  boolean toggleClimb;
  double targetPosition;
  double climbCalculate;
  ServoChannel climbServo;
  public ClimbSubsystem(ServoHub servoHub) {
    driveMotor = new TalonFX(41);//CHANGEEEEEEEEEEEEEEEEEE
    m_servoHub = servoHub;
    climbServo = servoHub.getServoChannel(ChannelId.kChannelId1);
    climbController = new PIDController(0.01, 0, 0);
    climbController.setTolerance(.001);
    toggleClimb = false;
    targetPosition = 0;
    climbCalculate = 0;
  }
  public void prepClimb(){
    targetPosition = 100;//CHANGEEEEEEEEEEEEEEEE
    climbServo.setEnabled(true);
    climbServo.setPulseWidth(0);
  }
  public void toggleClimb(){
    toggleClimb = !toggleClimb;
    if (toggleClimb){
      prepClimb();
    }
    else{
      climbServo.setPulseWidth(5000);
      climbServo.setEnabled(true);
      climb();
    }
  }

  public void climb(){
    targetPosition = 0;
    
  }
  public void setDriveMotor(double value){
    driveMotor.set(value);
  }
  @Override
  public void periodic() {
    if ((driveMotor.getPosition().getValueAsDouble()<targetPosition) && toggleClimb == true){
      //driveMotor.set(1);
    }
    else if ((driveMotor.getPosition().getValueAsDouble()>targetPosition) && toggleClimb == false){
      //driveMotor.set(-1);
    }
    else{
      //driveMotor.set(0);
    }
    //climbCalculate = climbController.calculate(driveMotor.getPosition().getValueAsDouble(), targetPosition);
    //driveMotor.set(climbCalculate);
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("climb motor Position", driveMotor.getPosition().getValueAsDouble());
  }
}
