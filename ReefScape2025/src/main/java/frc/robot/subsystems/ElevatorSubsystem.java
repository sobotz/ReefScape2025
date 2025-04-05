// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPosition;


public class ElevatorSubsystem extends SubsystemBase {
  SwerveSubsystem m_swerveSubsystem;
  VoltageOut m_voltReq;
  /** Elevator Motors */
  TalonFX elevatorMotor;
  TalonFX slaveMotor;
  CurrentLimitsConfigs limitConfigs;


  /** PID Controller for precise control */
  PIDController elevatorController;

  /** Mapping of Enum Positions to Heights */
  private final Map<ElevatorPosition, Double> positionMap;
  /** Motion Magic Control */

  private double elevatorPIDCalculation;
  private final MotionMagicVoltage motionMagicController;

  private ElevatorPosition targetPosition; 
  ElevatorPosition autoPlaceTargetElevatorPosition;
  /** PIDF Constants */
  /** Manual Control Mode */
  //private boolean manualMode;
  boolean once;
  boolean atTargetPosition;
  double previousElevatorError;
  double atPositionCount;
  boolean intakeFinishMode;
  boolean isAuto;
  
  /*Motion Magic */
  TalonFXConfiguration talonFXConfiguration;
  Slot0Configs slot0Configs;
  MotionMagicConfigs motionMagicConfigs;
  
  MotionMagicVoltage m_voltageRequest;
  SysIdRoutine m_sysIdRoutine;
  
  
    public ElevatorSubsystem(SwerveSubsystem swerveSubsystem) {
      /** Initialize position mappings */
      m_voltReq = new VoltageOut(0);
      isAuto = false;
      intakeFinishMode = false;
      m_swerveSubsystem = swerveSubsystem;
      positionMap = new HashMap<ElevatorPosition, Double>(){{
        put(ElevatorPosition.DEFAULT, ElevatorConstants.DEFAULT);
        put(ElevatorPosition.ALGAETEMP, ElevatorConstants.ALGAETEMP);
        put(ElevatorPosition.INTAKE, ElevatorConstants.INTAKE);
        put(ElevatorPosition.CLIMB, ElevatorConstants.CLIMB);
        put(ElevatorPosition.FLOORALGAE,ElevatorConstants.FLOORALGAE);
        put(ElevatorPosition.PROCESSOR,ElevatorConstants.PROCESSOR);
        put(ElevatorPosition.LOWESTALGAE,ElevatorConstants.LOWESTALGAE);
        put(ElevatorPosition.LOWERALGAE,ElevatorConstants.LOWERALGAE);
        put(ElevatorPosition.MIDALGAE, ElevatorConstants.MIDALGAE);
        put(ElevatorPosition.HIGHERALGAE, ElevatorConstants.HIGHERALGAE);
        put(ElevatorPosition.BARGE, ElevatorConstants.BARGE);
        put(ElevatorPosition.L1, ElevatorConstants.L1);
        put(ElevatorPosition.L2, ElevatorConstants.L2);
        put(ElevatorPosition.L3, ElevatorConstants.L3);
        put(ElevatorPosition.L4, ElevatorConstants.L4);
        put(ElevatorPosition.TEMPPOSITION, ElevatorConstants.TEMPPOSITION);
      }};
  
      /** Initialize motors */
      elevatorMotor = new TalonFX(15);
      elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
      slaveMotor = new TalonFX(14);
      slaveMotor.setNeutralMode(NeutralModeValue.Brake);
  
      /** Configure motors */
      limitConfigs = new CurrentLimitsConfigs();
      limitConfigs.StatorCurrentLimit = 80;
      limitConfigs.SupplyCurrentLimit = 70;
      limitConfigs.StatorCurrentLimitEnable = true;
      limitConfigs.SupplyCurrentLimitEnable = true;
      
      m_sysIdRoutine =
        new SysIdRoutine(
          new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())
          ),
        
          new SysIdRoutine.Mechanism(
          (volts) -> elevatorMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
          null,
          this
        )
      );
      
      
      /*Motion Magic Configurations */
      // talonFXConfiguration = new TalonFXConfiguration();
      // talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 42.75;
      // talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      // slot0Configs = talonFXConfiguration.Slot0;
      // slot0Configs.kS = 0.25; //CHANGE ALL --->
      // slot0Configs.kV = 0.12;
      // slot0Configs.kA = 0.01;
      // slot0Configs.kP = 4.8;
      // slot0Configs.kI = 0.0;
      // slot0Configs.kD = 0.0;
      
      // motionMagicConfigs = talonFXConfiguration.MotionMagic;
      // motionMagicConfigs.MotionMagicCruiseVelocity = 100; //CHANGE ALL --->
      // motionMagicConfigs.MotionMagicAcceleration = 400;
      
      // motionMagicConfigs.MotionMagicJerk = 1600;
  
      /*Apply Configs*/
      //elevatorMotor.getConfigurator().apply(talonFXConfiguration);
      //slaveMotor.getConfigurator().apply(talonFXConfiguration);
  
      //Voltage Request
      
    m_voltageRequest = new MotionMagicVoltage(0);
    //elevatorMotor.getConfigurator().apply(limitConfigs);
    //slaveMotor.getConfigurator().apply(limitConfigs);


    
    previousElevatorError = 0;
    atPositionCount = 0;
    // enable stator current limit
    motionMagicController = new MotionMagicVoltage(0.0);
    configureMotionMagic();
    
    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI,ElevatorConstants.kD);
    elevatorController.setTolerance(0.002); 
    elevatorPIDCalculation = 0; 
    
    targetPosition = ElevatorPosition.DEFAULT;
    autoPlaceTargetElevatorPosition = ElevatorPosition.L4;
    once = true;
    atTargetPosition = false;
  }

  public double getElevatorSensorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }
  public void intakeFinish(){
    intakeFinishMode = true;
  }
  public void setIsAuto(boolean value){
    isAuto = value;
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.dynamic(direction);
  }

  public boolean elevatorAtTargetPosition() {
    if (Math.abs(elevatorController.getError())<0.43 && Math.abs(elevatorPIDCalculation)<0.07){
      atTargetPosition = false;
      if (Math.abs(previousElevatorError - elevatorController.getError()) < 0.07){//(Math.abs(clawController.getError())<0.13) && Math.abs(clawPIDCalculation)<0.0023){
        atPositionCount += 1;
      }
      else{
        atPositionCount = 0;
      }
      previousElevatorError = elevatorController.getError();
      if (atPositionCount > 1){
        atTargetPosition = true;
        atPositionCount = 0;
        //System.out.println("elevatorAtPosition");
        return true;
      }
      else{
        atTargetPosition = false;
        return false;
      }
    }
    else{
      previousElevatorError = elevatorController.getError();
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
  private void configureMotionMagic() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kG = ElevatorConstants.kG;
    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kA = ElevatorConstants.kA;
    

    // Motion Magic velocity/acceleration
    config.MotionMagic.MotionMagicCruiseVelocity = 100;  // in rotations/second
    config.MotionMagic.MotionMagicAcceleration = 200;    // in rotations/second^2
    config.MotionMagic.MotionMagicJerk = 1800;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.StatorCurrentLimit = 80;
    
    
    

    
    // Apply configuration
    elevatorMotor.getConfigurator().apply(config);
    slaveMotor.getConfigurator().apply(config);
}
public void moveToPositionMM(ElevatorPosition position) {
  double targetRotations = positionMap.get(position); // Assuming this is in rotations\
  elevatorMotor.setControl(motionMagicController.withPosition(targetRotations));
  slaveMotor.setControl(motionMagicController.withPosition(-targetRotations));
  
  
  
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
    //System.out.println(targetPosition);
    if (once){
      elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
      slaveMotor.setNeutralMode(NeutralModeValue.Brake);
      once = false;
    }
    
    SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Target", positionMap.get(targetPosition));
    SmartDashboard.putNumber("Elevator Error", elevatorMotor.getPosition().getValueAsDouble() - positionMap.get(targetPosition));
    SmartDashboard.putNumber("elevator voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("elevator current", elevatorMotor.getStatorCurrent().getValueAsDouble());
    
    //SmartDashboard.putBoolean("Manual Mode", manualMode);
    //System.out.println("Manual Mode: " + manualMode);
    
   elevatorPIDCalculation = elevatorController.calculate(getElevatorSensorPosition(), positionMap.get(targetPosition));
    
   moveToPositionMM(targetPosition);

    if (isAuto){
      if (elevatorAtTargetPosition()){
        isAuto = false;
      }
      if (elevatorPIDCalculation>0.6){
        elevatorPIDCalculation = 0.6;
      }
      else if (elevatorPIDCalculation<-0.6){
        elevatorPIDCalculation = -0.6;
      }
    } 
    
    if (elevatorPIDCalculation > 0.9){
      elevatorPIDCalculation = 0.9;
    }
    else if (elevatorPIDCalculation < -0.9){
      elevatorPIDCalculation = -0.9;
    }
    if (Math.abs(elevatorPIDCalculation)<0.03){
      elevatorPIDCalculation *= 1.6;
    }
    else if (Math.abs(elevatorPIDCalculation) < 0.04){
      elevatorPIDCalculation *= 1.4;
    }
    

    // if (intakeFinishMode){
    //   if (elevatorPIDCalculation>0.7){
    //     elevatorPIDCalculation = 0.7;
    //   }
    //   else if (elevatorPIDCalculation < -0.7){
    //     elevatorPIDCalculation = -0.7;
    //   }
    //   if (elevatorAtTargetPosition()){
    //     intakeFinishMode = false;
    //   }
    // }
    
    //elevatorMotor.set(0.03);
    //slaveMotor.set(-0.03);
    //SmartDashboard.putNumber("Elevator Calculation", elevatorPIDCalculation);
    //SmartDashboard.putNumber("Elevator Error", elevatorController.getError());
    //SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
    
    //elevatorMotor.set((elevatorPIDCalculation));
    //slaveMotor.set((-elevatorPIDCalculation));
    
    /*Motion Magic Set Target Position */
   
    //elevatorMotor.setControl(m_voltageRequest.withPosition(x));
    ////slaveMotor.setControl(m_voltageRequest.withPosition(x));
    
    //System.out.println(atTargetPosition);
  }
}
