// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
  
  /** Elevator Motors */
  TalonFX elevatorMotor;
  TalonFX slaveMotor;
  CurrentLimitsConfigs limitConfigs;


  /** PID Controller for precise control */
  private final PIDController elevatorController;

  /** Mapping of Enum Positions to Heights */
  private final Map<ElevatorPosition, Double> positionMap;

  /** Motion Magic Control */

  private double elevatorPIDCalculation;
  private ElevatorPosition targetPosition; 
  /** PIDF Constants */
  /** Manual Control Mode */
  private boolean manualMode;
  boolean once;
  boolean atTargetPosition;

  public ElevatorSubsystem() {
    /** Initialize position mappings */
    
    positionMap = new HashMap<ElevatorPosition, Double>(){{
      put(ElevatorPosition.DEFAULT, ElevatorConstants.DEFAULT);
      put(ElevatorPosition.INTAKE, ElevatorConstants.INTAKE);
      put(ElevatorPosition.FLOORALGAE,ElevatorConstants.FLOORALGAE);
      put(ElevatorPosition.LOWERALGAE,ElevatorConstants.LOWERALGAE);
      put(ElevatorPosition.HIGHERALGAE, ElevatorConstants.HIGHERALGAE);
      put(ElevatorPosition.BARGE, ElevatorConstants.BARGE);
      put(ElevatorPosition.L1, ElevatorConstants.L1);
      put(ElevatorPosition.L2, ElevatorConstants.L2);
      put(ElevatorPosition.L3, ElevatorConstants.L3);
      put(ElevatorPosition.L4, ElevatorConstants.L4);
    }};

    /** Initialize motors */
    elevatorMotor = new TalonFX(15);
    slaveMotor = new TalonFX(14);

    /** Configure motors */
    limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 40;
    limitConfigs.StatorCurrentLimitEnable = true;
    
    elevatorMotor.getConfigurator().apply(limitConfigs);
    slaveMotor.getConfigurator().apply(limitConfigs);
    

    // enable stator current limit

    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI,ElevatorConstants.kD);  // Tune these values as needed
    elevatorController.setTolerance(0.005); 
    elevatorPIDCalculation = 0; 
    targetPosition = ElevatorPosition.DEFAULT;
    once = true;
    atTargetPosition = false;
  }

  public double getElevatorSensorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean elevatorAtTargetPosition() {
    if (Math.abs(elevatorController.getError())<0.17){
      return true;
    }
    else{
      return false;
    }
  }

  public void setElevatorTargetPosition(ElevatorPosition position) {
    targetPosition = position;
  }

  @Override
  public void periodic() {
    if (once){
      elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
      slaveMotor.setNeutralMode(NeutralModeValue.Brake);
      once = false;
    }
    //SmartDashboard.putBoolean("Manual Mode", manualMode);
    //System.out.println("Manual Mode: " + manualMode);
    
    elevatorPIDCalculation = elevatorController.calculate(getElevatorSensorPosition(), positionMap.get(targetPosition));
    
    //elevatorMotor.set(0.03);
    //slaveMotor.set(-0.03);
    if (!elevatorController.atSetpoint()) {
      //System.out.println(elevatorController.getError());
      //System.out.println(elevatorPIDCalculation);
      elevatorMotor.set((elevatorPIDCalculation)*0.5);
      slaveMotor.set((-elevatorPIDCalculation)*0.5);
    } 
    if (Math.abs(elevatorController.getError())<0.3){
      atTargetPosition = true;
    }
    else{
      atTargetPosition = false;
    }
    //System.out.println(atTargetPosition);
  }
}
