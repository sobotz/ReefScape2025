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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
  SwerveSubsystem m_swerveSubsystem;

  /** Elevator Motors */
  TalonFX elevatorMotor;
  TalonFX slaveMotor;
  CurrentLimitsConfigs limitConfigs;


  /** PID Controller for precise control */
  ProfiledPIDController elevatorController;

  /** Mapping of Enum Positions to Heights */
  private final Map<ElevatorPosition, Double> positionMap;

  /** Motion Magic Control */

  private double elevatorPIDCalculation;
  private ElevatorPosition targetPosition; 
  ElevatorPosition autoPlaceTargetElevatorPosition;
  /** PIDF Constants */
  /** Manual Control Mode */
  private boolean manualMode;
  boolean once;
  boolean atTargetPosition;
  double previousElevatorError;
  double atPositionCount;

 

  

  

  public ElevatorSubsystem(SwerveSubsystem swerveSubsystem) {
    /** Initialize position mappings */
    m_swerveSubsystem = swerveSubsystem;

    
    
    positionMap = new HashMap<ElevatorPosition, Double>(){{
      put(ElevatorPosition.DEFAULT, ElevatorConstants.DEFAULT);
      put(ElevatorPosition.INTAKE, ElevatorConstants.INTAKE);
      put(ElevatorPosition.FLOORALGAE,ElevatorConstants.FLOORALGAE);
      put(ElevatorPosition.PROCESSOR,ElevatorConstants.PROCESSOR);
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
    
    previousElevatorError = 0;
    atPositionCount = 0;
    // enable stator current limit

    elevatorController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI,ElevatorConstants.kD, new TrapezoidProfile.Constraints(6, 2));  // Tune these values as needed
    elevatorController.setTolerance(0.002); 
    elevatorPIDCalculation = 0; 
    targetPosition = ElevatorPosition.DEFAULT;
    autoPlaceTargetElevatorPosition = ElevatorPosition.DEFAULT;
    once = true;
    atTargetPosition = false;
  }

  public double getElevatorSensorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean elevatorAtTargetPosition() {
    /*if (Math.abs(elevatorController.getError())<0.26 && Math.abs(elevatorPIDCalculation)<0.035){
      atTargetPosition = true;
      return true;
    }
    else{
      atTargetPosition = false;
      return false;
    }*/
    atTargetPosition = false;
    if (Math.abs(previousElevatorError) - Math.abs(elevatorController.getPositionError()) < 0.01){//(Math.abs(clawController.getError())<0.13) && Math.abs(clawPIDCalculation)<0.0023){
      atPositionCount += 1;
    }
    else{
      atPositionCount = 0;
    }
    previousElevatorError = elevatorController.getPositionError();
    if (atPositionCount > 2){
      atTargetPosition = true;
      atPositionCount = 0;
      return true;
    }
    else{
      atTargetPosition = false;
      return false;
    }
  }
  public double getElevatorPIDCalculate(){
    return elevatorPIDCalculation;
  }
  public ElevatorPosition getTargetElevatorPosition(){
    return targetPosition;
  }
  
  public void setElevatorTargetPosition(ElevatorPosition position) {
    targetPosition = position;
  }
  public ElevatorPosition PredictedAlgaeElevatorPosition(){
    if(m_swerveSubsystem.getTargetID()%2 == 0){
      return ElevatorPosition.LOWERALGAE;
    }
    else{
      return ElevatorPosition.HIGHERALGAE;
    }
  }
  public Map<ElevatorPosition, Double> getPositionMap(){
    return positionMap;
  }
  

  public ElevatorPosition getAutoPlacePosition(){
    return autoPlaceTargetElevatorPosition;
  }
  public void setAutoPlaceClawTargetPosition(ElevatorPosition position){
    autoPlaceTargetElevatorPosition = position;
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
    SmartDashboard.putNumber("Elevator Calculation", elevatorPIDCalculation);
    SmartDashboard.putNumber("Elevator Error", elevatorController.getPositionError());
    
    elevatorMotor.set((elevatorPIDCalculation));
    slaveMotor.set((-elevatorPIDCalculation));
    
    
    //System.out.println(atTargetPosition);
  }
}
