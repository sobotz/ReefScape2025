// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
  CurrentLimitsConfigs limitConfigs;
  CANcoder clawSensor;
  PIDController clawController;
  PIDController retainAlgaeController;
  Map<ClawPosition, Double> clawPositionMap;
  ClawPosition clawTargetPosition;
  ClawPosition autoPlaceClawTargetPosition;

  double clawPIDCalculation;
  double algaeRetainPosition;
  boolean atTarget;
  boolean hasCoral;
  boolean hasAlgae;
  boolean driveMotorIsControlled;
  public ClawSubsystem() {
    wristMotor = new TalonFX(18);//CHANGE
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;
    wristMotor.getConfigurator().apply(limitConfigs);
    clawDriveMotor = new TalonFX(16);
    clawDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    
    clawSensor = new CANcoder(17);//CHANGE

    clawController = new PIDController(0.010, 0.0000, 0.000);//P0.0155 d 0.00017
    //clawController.enableContinuousInput(0,360);
    clawController.setTolerance(0.0);

    retainAlgaeController = new PIDController(0.001,0, 0);

    clawPositionMap = new HashMap<ClawPosition, Double>(){{
      put(ClawPosition.DEFAULT, ClawConstants.DEFAULT);
      put(ClawPosition.FLOORALGAE,ClawConstants.FLOORALGAE);
      put(ClawPosition.FACINGDOWNREEFALGAE, ClawConstants.FACINGUPREEFALGAE);
      put(ClawPosition.FACINGUPREEFALGAE,ClawConstants.FACINGUPREEFALGAE);
      put(ClawPosition.BARGE,ClawConstants.BARGE);
      put(ClawPosition.INTAKE, ClawConstants.INTAKE);
      put(ClawPosition.L1, ClawConstants.L1);
      put(ClawPosition.L2, ClawConstants.L2);
      put(ClawPosition.L3, ClawConstants.L3);
      put(ClawPosition.L4, ClawConstants.L4);
    }};
    clawTargetPosition = ClawPosition.DEFAULT;
    autoPlaceClawTargetPosition = ClawPosition.DEFAULT;
    clawPIDCalculation = 0;
    atTarget = false;
    hasCoral = false;
    hasAlgae = false;
    driveMotorIsControlled = false;
    algaeRetainPosition = 0;
  }

  public double getClawSensorPosition(){
    return (-1 * ((clawSensor.getPosition().getValueAsDouble()*360)-75.02148438));
  }

  public boolean clawAtTargetPosition(){
    if ((Math.abs(clawController.getError())<0.2) && clawPIDCalculation<0.008){
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
  public ClawPosition getAutoPlacePosition(){
    return autoPlaceClawTargetPosition;
  }
  public void setAutoPlaceClawTargetPosition(ClawPosition position){
    autoPlaceClawTargetPosition = position;
  }
  public void setDriveMotor(double value){
    
    clawDriveMotor.set(value);
    if (value == 0){
      driveMotorIsControlled = false;
    }
    else{
      driveMotorIsControlled = true;
    }
  }
  public double getClawDriveMotorPosition(){
    return clawDriveMotor.getPosition().getValueAsDouble();
  }
  
  public double getClawPIDCalcuate(){
    return clawPIDCalculation;
  }
  public void setHasAlgae(boolean hasAlgae){
    this.hasAlgae = hasAlgae;
    if (this.hasAlgae == true){
      clawPositionMap.put(ClawPosition.DEFAULT,ClawConstants.HASALGAEDEFAULT);
    }
    else{
      clawPositionMap.put(ClawPosition.DEFAULT,ClawConstants.DEFAULT);
    }
  }
  public void setAlgaeRetainPosition(){
    algaeRetainPosition = getClawDriveMotorPosition();
  }


  @Override
  public void periodic() {
    
    //System.out.println(getClawSensorPosition());
    SmartDashboard.putNumber("clawSensorPosition",getClawSensorPosition());
    //System.out.println(getClawSensorPosition());
    // This method will be called once per scheduler run
    clawPIDCalculation = 1.5*clawController.calculate(getClawSensorPosition(), clawPositionMap.get(clawTargetPosition));
    //System.out.println(clawPositionMap.get(clawTargetPosition));
    //SmartDashboard.putNumber("getTargetPosition",clawPositionMap.get(clawTargetPosition));
    SmartDashboard.putNumber("Claw Error",clawController.getError());
    SmartDashboard.putNumber("Claw Calculation",clawPIDCalculation);
    //System.out.println(clawController.getError());
    //System.out.println(clawPIDCalculation);
    if (!clawController.atSetpoint()){
      if (clawPIDCalculation > 0.75){
        clawPIDCalculation = 0.75;
      }
      else if (clawPIDCalculation < -0.75){
        clawPIDCalculation = -0.75;
      }
      // if (Math.abs(clawPIDCalculation)<0.1){
      //   clawPIDCalculation = clawPIDCalculation * 1.032;
      // }
      /*if (Math.abs(clawPIDCalculation)<0.1){
        clawPIDCalculation = clawPIDCalculation * .95;
      }*/
      if (Math.abs(clawPIDCalculation)<0.01){
        clawPIDCalculation = clawPIDCalculation * 4;
        //System.out.println("activated");
      }
      else if (Math.abs(clawPIDCalculation)<0.02){
        clawPIDCalculation = clawPIDCalculation * 1.7;
        //System.out.println("first Activation");
      }
      /*else if (Math.abs(clawPIDCalculation)<0.025){
        clawPIDCalculation = clawPIDCalculation * 1.02;
      }*/
      
      // if (clawPIDCalculation < 0.05 && clawPIDCalculation > 0){
      //   clawPIDCalculation ;
      // }
      // else if (clawPIDCalculation > -0.05 && clawPIDCalculation < 0){
      //   clawPIDCalculation *= 1.1;
      // }
      wristMotor.set(clawPIDCalculation);
    }
    else{
      wristMotor.set(0);      
    }
    /*if (hasAlgae && !driveMotorIsControlled){
      clawDriveMotor.set(retainAlgaeController.calculate(getClawDriveMotorPosition(),algaeRetainPosition));
    }*/
  }
}
