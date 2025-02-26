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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawPosition;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  TalonFX wristMotor;
  TalonFX clawDriveMotor;
  CANcoder clawSensor;
  PIDController clawController;
  Map<ClawPosition, Double> clawPositionMap;
  ClawPosition clawTargetPosition;

  double clawPIDCalculation;
  boolean atTarget;
  public ClawSubsystem() {
    wristMotor = new TalonFX(18);//CHANGE
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    clawDriveMotor = new TalonFX(16);
    clawSensor = new CANcoder(17);//CHANGE

    clawController = new PIDController(0.006, 0, 0);
    clawController.enableContinuousInput(0,360);
    clawController.setTolerance(0.001);

    clawPositionMap = new HashMap<ClawPosition, Double>(){{
      put(ClawPosition.DEFAULT, ClawConstants.defaultSensorPosition);
      put(ClawPosition.INTAKE, ClawConstants.intakeSensorPosition);
      put(ClawPosition.L1, ClawConstants.L1SensorPosition);
      put(ClawPosition.L2, ClawConstants.L2SensorPosition);
      put(ClawPosition.L3, ClawConstants.L3SensorPosition);
      put(ClawPosition.L4, ClawConstants.L4SensorPosition);
    }};
    clawTargetPosition = ClawPosition.DEFAULT;
    clawPIDCalculation = 0;
    atTarget = false;
  }
  
  public double getClawSensorPosition(){
    return 360-((((clawSensor.getPosition().getValueAsDouble()%1)+1)%1)*360);
  }

  public boolean clawAtTargetPosition(){
    if (clawController.getError()<0.2){
      atTarget = true;
      return true;
    }
    else{
      atTarget = false;
      return false;
    }
  }
  
  public void setClawTargetPosition(ClawPosition position){
    clawTargetPosition = position;
  }
  public void setDriveMotor(double value){
    clawDriveMotor.set(value);
  }
  

  @Override
  public void periodic() {
    //System.out.println(getClawSensorPosition());
    // This method will be called once per scheduler run
    clawPIDCalculation = clawController.calculate(getClawSensorPosition(), clawPositionMap.get(clawTargetPosition));
    System.out.println(clawPositionMap.get(clawTargetPosition));
    //SmartDashboard.putNumber("getTargetPosition",clawPositionMap.get(clawTargetPosition));
    if (!clawController.atSetpoint()){
      wristMotor.set(clawPIDCalculation);
    }
    else{
      wristMotor.set(0);
    }
  }
}
