// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFXS driveMotor;

  public IntakeSubsystem() {
    driveMotor = new TalonFXS(13);
    TalonFXSConfiguration toConfigure = new TalonFXSConfiguration();
    toConfigure.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    // Use the connected quadrature encoder for any closed loop control. The
    // regular position and velocity getters will return position/velocity
    // as retrieved from the connected quadrature encoder.
    toConfigure.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Quadrature;

    // Store the result of the config apply to variable, the user can check this
    // to make sure the config apply actually succeeded.
    driveMotor.getConfigurator().apply(toConfigure);
  }
  public void setDriveMotor(double value){
    System.out.println("drive");
    driveMotor.set(value);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
