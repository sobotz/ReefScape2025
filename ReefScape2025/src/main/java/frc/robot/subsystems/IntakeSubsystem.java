// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFX intakeMotor;

  ServoHub servoHub;
  public IntakeSubsystem() {
    //Change the ID!!!!!!
    intakeMotor =  new TalonFX(0);

    //Swyft Servo Thing (Change ID as well)
    servoHub = new ServoHub(1);

  }
  public void runIntake(){
    //Default intakeMotor speed (0.5)
    intakeMotor.set(0.5);
  }
  public void stopIntake(){
    intakeMotor.set(0);
  }
  public void dropIntake(){
    //Turning on the servo, setting it 180 degrees and disabling immediatly after.
    //Turns off the intakeMotor to 0 to prevent wasted battery usage.

    intakeMotor.set(0);
    //Change the servo channel as well
    (servoHub.getServoChannel(ChannelId.kChannelId0)).setEnabled(true); 
    (servoHub.getServoChannel(ChannelId.kChannelId0)).setPulseWidth(1500);
    (servoHub.getServoChannel(ChannelId.kChannelId0)).setEnabled(false); 
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
