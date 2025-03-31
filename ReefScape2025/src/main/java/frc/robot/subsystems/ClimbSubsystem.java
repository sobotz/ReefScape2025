// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
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
  boolean testServo;
  Timer timer;
  boolean bufferPrep;
  SwerveSubsystem m_swerveSubsystem;
  CurrentLimitsConfigs limitConfigs;
  
  public ClimbSubsystem(SwerveSubsystem swerveSubsystem, ServoHub servoHub) {
    limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 70;
    limitConfigs.StatorCurrentLimitEnable = true;
    m_swerveSubsystem = swerveSubsystem;
    bufferPrep = false;
    timer = new Timer();
    testServo = false;
    driveMotor = new TalonFX(41);//CHANGEEEEEEEEEEEEEEEEEE
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.getConfigurator().apply(limitConfigs);
    m_servoHub = servoHub;
    climbServo = servoHub.getServoChannel(ChannelId.kChannelId1);
    climbController = new PIDController(0.01, 0, 0);
    climbController.setTolerance(.001);
    toggleClimb = false;
    targetPosition = 0;
    climbCalculate = 0;
  }
  public void testServo( boolean value){
    testServo = value;
  }
  public void prepClimb(){
    //m_swerveSubsystem.setDriveCommandDisabled(false);
    timer.start();
    testServo = false;//CHANGEEEEEEEEEEEEEEEE
    //climbServo.setEnabled(true);
    //climbServo.setPulseWidth(0);
  }
  public void toggleClimb(){
    timer.reset();
    timer.stop();
    toggleClimb = !toggleClimb;
    if (toggleClimb){
      prepClimb();
    }
    else{
      //climbServo.setPulseWidth(5000);
      //climbServo.setEnabled(true);
      climb();
    }
  }

  public void climb(){
    //m_swerveSubsystem.setDriveCommandDisabled(true);
    bufferPrep = false;
    testServo = true;
    //System.out.println("working");
    targetPosition = 20;
  }
  public void setDriveMotor(double value){
    driveMotor.set(value);
  }
  @Override
  public void periodic() {
    if (timer.get()>1){
      targetPosition = -83;
      bufferPrep = true;
    }
    if (toggleClimb && bufferPrep){
      if (driveMotor.getPosition().getValueAsDouble()>targetPosition + 1){
        driveMotor.set(-1);
      }
      else if (driveMotor.getPosition().getValueAsDouble()<targetPosition-2){
        driveMotor.set(0.5);
      }
      else{
        driveMotor.set(0);
      }
    
    }
    else if (!toggleClimb){
      if (driveMotor.getPosition().getValueAsDouble()<targetPosition - 0.3){
        driveMotor.set(1);
      }
      else{
        driveMotor.set(0);
      }
    }
    if (testServo){
      climbServo.setPulseWidth(2000);
    }
    else{
      climbServo.setEnabled(true);
      climbServo.setPowered(true);
      climbServo.setPulseWidth(1500);
    }
    
    
    //SmartDashboard.putNumber("Climb Servo", climbServo.getPulseWidth());
    //climbCalculate = climbController.calculate(driveMotor.getPosition().getValueAsDouble(), targetPosition);
    //driveMotor.set(climbCalculate);
    // This method will be called once per scheduler run
    
    //SmartDashboard.putNumber("climb motor Position", driveMotor.getPosition().getValueAsDouble());
  }
}
