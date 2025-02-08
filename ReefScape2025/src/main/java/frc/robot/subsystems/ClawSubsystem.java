// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawPosition;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  TalonFX clawMotor;
  CANcoder clawSensor;
  PIDController clawController;
  Map<ClawPosition, Double> positionMap;
  ClawPosition targetPosition;

  double clawPIDCalculation;
  public ClawSubsystem() {
    clawMotor = new TalonFX(20);//CHANGE
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
    clawSensor = new CANcoder(21);//CHANGE
    clawController = new PIDController(0, 0, 0);
    clawController.setTolerance(0.001);

    positionMap = new HashMap<ClawPosition, Double>(){{
      put(ClawPosition.INTAKE, ClawConstants.intakeSensorPosition);
      put(ClawPosition.L1, ClawConstants.L1SensorPosition);
      put(ClawPosition.L2, ClawConstants.L2SensorPosition);
      put(ClawPosition.L3, ClawConstants.L3SensorPosition);
      put(ClawPosition.L4, ClawConstants.L4SensorPosition);
    }};
    targetPosition = ClawPosition.INTAKE;
    clawPIDCalculation = 0;
  }

  public double getClawSensorPosition(){
    return clawSensor.getPosition().getValueAsDouble();
  }

  public boolean clawAtTargetPosition(){
    return clawController.atSetpoint();
  }
  
  public void setClawTargetPosition(ClawPosition position){
    targetPosition = position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    clawPIDCalculation = clawController.calculate(getClawSensorPosition(), (double)positionMap.get(targetPosition));
    if (clawController.atSetpoint() == false){
      clawMotor.set(clawPIDCalculation);
    }
    else{
      clawMotor.set(0);
    }
  }
}
