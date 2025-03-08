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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawPosition;
import frc.robot.Constants.ElevatorPosition;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  TalonFX wristMotor;
  TalonFX clawDriveMotor;
  DigitalInput proxSensor;
  CurrentLimitsConfigs limitConfigs;
  CANcoder clawSensor;
  PIDController clawController;
  PIDController retainAlgaeController;
  Map<ClawPosition, Double> clawPositionMap;
  ClawPosition clawTargetPosition;
  ClawPosition autoPlaceClawTargetPosition;
  Timer timer;

  double clawPIDCalculation;
  double algaeRetainPosition;
  double originalCanCoderPosition;
  boolean atTarget;
  boolean hasCoral;
  boolean hasAlgae;
  boolean driveMotorIsControlled;
  boolean once;
  double originalWristSensorPosition;
  double previousClawError;
  double atPositionCount;
  boolean proxTripped;
  boolean reefCoralPlacementButton;
  boolean reefAlgaeGrabButton;
  boolean toggleProcessor;
  boolean toggleBarge;
  double driveMotorCurrent;
  Timer intakeTimer;



  public ClawSubsystem() {
    intakeTimer = new Timer();
    driveMotorCurrent = 0;
    toggleProcessor = false;
    toggleBarge = false;
    reefCoralPlacementButton = true;
    reefAlgaeGrabButton = true;
    timer = new Timer();
    proxSensor = new DigitalInput(9);//CHANGE
    proxTripped = false;
    wristMotor = new TalonFX(18);
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
    limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;
    wristMotor.getConfigurator().apply(limitConfigs);
    clawDriveMotor = new TalonFX(16);
    clawDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    
    
    clawSensor = new CANcoder(17);
    
    clawController = new PIDController(0.012, 0.0000, 0.000);
    //clawController.enableContinuousInput(0,360);
    clawController.setTolerance(0.01);

    retainAlgaeController = new PIDController(0.012,0, 0);

    clawPositionMap = new HashMap<ClawPosition, Double>(){{
      put(ClawPosition.DEFAULT, ClawConstants.DEFAULT);
      put(ClawPosition.FLOORALGAE,ClawConstants.FLOORALGAE);
      put(ClawPosition.PROCESSOR,ClawConstants.PROCESSOR);
      put(ClawPosition.FACINGDOWNREEFALGAE, ClawConstants.FACINGDOWNREEFALGAE);
      put(ClawPosition.FACINGUPREEFALGAE,ClawConstants.FACINGUPREEFALGAE);
      put(ClawPosition.BARGE,ClawConstants.BARGE);
      put(ClawPosition.INTAKE, ClawConstants.INTAKE);
      put(ClawPosition.L1, ClawConstants.L1);
      put(ClawPosition.L2, ClawConstants.L2);
      put(ClawPosition.L3, ClawConstants.L3);
      put(ClawPosition.L4, ClawConstants.L4);
      put(ClawPosition.TEMPPOSITION, ClawConstants.TEMPPOSITION);
    }};
    clawTargetPosition = ClawPosition.DEFAULT;
    autoPlaceClawTargetPosition = ClawPosition.L1;
    clawPIDCalculation = 0;
    atTarget = false;
    hasCoral = false;
    hasAlgae = false;
    driveMotorIsControlled = false;
    algaeRetainPosition = 0;
    once = true;
    originalCanCoderPosition = 0;
    originalWristSensorPosition = 0;
    previousClawError = 0;
    atPositionCount = 0;
  }
  

  public double getClawSensorPosition(){
    return (((((wristMotor.getPosition().getValueAsDouble() - originalWristSensorPosition)*360)/82.1333333)-90));
  }


  public Map<ClawPosition, Double> getPositionMap(){
    return clawPositionMap;
  }
  public void toggleProcessor(){
    toggleProcessor = !toggleProcessor;
  }
  public boolean getToggleProcessor(){
    return toggleProcessor;
  }
  public void toggleBarge(){
    toggleBarge = !toggleBarge;
  }
  public boolean getToggleBarge(){
    return toggleBarge;
  }

  public boolean clawAtTargetPosition(){
    if ((Math.abs(clawController.getError())<0.21) && (Math.abs(clawPIDCalculation)<0.03)){
      //System.out.println("inrange");
      if (Math.abs(previousClawError - clawController.getError()) <0.06){//(Math.abs(clawController.getError())<0.13) && Math.abs(clawPIDCalculation)<0.0023){
        atPositionCount += 1;
      }  
      else{
        atPositionCount = 0;
      }

      previousClawError = clawController.getError();
      if (atPositionCount > 1){
        //System.out.println("atPositionCLaw");
        atTarget = true;
        atPositionCount = 0;
        return true;
      }
      else{
        atTarget = false;
        return false;
      }
    }
    else{
      previousClawError = clawController.getError();
      atTarget = false;
      return false;
    }
    
  }
  public boolean getReefCoralPlacementButton(){
    return reefCoralPlacementButton;
  }
  public boolean getReefAlgaeGrabButton(){
    return reefAlgaeGrabButton;
  }
  public void toggleReefCoralPlacementButton(){
    reefCoralPlacementButton = !reefCoralPlacementButton;
  }
  public void toggleReefAlgaeGrabButton(){
    reefAlgaeGrabButton = !reefAlgaeGrabButton;
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
      if (hasCoral && !hasAlgae){
        intakeTimer.start();
        clawDriveMotor.set(0.13);
      }
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
  public boolean hasItem(){
    return (hasAlgae || hasCoral);
  }
  public boolean getHasAlgae(){
    return hasAlgae;
  }
  public void setHasCoral(boolean hasCoral){
    this.hasCoral = hasCoral;
  }
  public boolean getHasCoral(){
    return hasCoral;
  }
  public void setAlgaeRetainPosition(){
    algaeRetainPosition = getClawDriveMotorPosition()+1.2;
  }

  public boolean getProximityTripped(){
    return proxTripped;
  }
  public double getDriveMotorCurrent(){
    return driveMotorCurrent;
  }


  @Override
  public void periodic() {
    driveMotorCurrent = clawDriveMotor.getTorqueCurrent().getValueAsDouble();
    proxTripped = !proxSensor.get();
    if (once){
      originalWristSensorPosition = wristMotor.getPosition().getValueAsDouble();
      once = false;  
    }
    if (intakeTimer.get()>0.7){
      intakeTimer.reset();
      intakeTimer.stop();
      clawDriveMotor.set(0);
    }
    SmartDashboard.putBoolean("clawProx", proxTripped);
    //System.out.println(wristMotor.getPosition().getValueAsDouble() * 360);
    //System.out.println(wristMotor.getPosition().getValueAsDouble() - originalWristSensorPosition);
    //System.out.println(getClawSensorPosition());
    SmartDashboard.putNumber("clawSensorPosition",getClawSensorPosition());
    //System.out.println(getClawSensorPosition());
    // This method will be called once per scheduler run
    clawPIDCalculation = 1.5 * clawController.calculate(getClawSensorPosition(), clawPositionMap.get(clawTargetPosition));
    //System.out.println(clawPositionMap.get(clawTargetPosition));
    //SmartDashboard.putNumber("getTargetPosition",clawPositionMap.get(clawTargetPosition));
    SmartDashboard.putNumber("Claw Error",clawController.getError());
    SmartDashboard.putNumber("Claw Calculation",clawPIDCalculation);
    SmartDashboard.putNumber("clawDrive Current",driveMotorCurrent);
    //System.out.println(clawController.getError());
    //System.out.println(clawPIDCalculation);
    //System.out.println((-1 * clawSensor.getAbsolutePosition().getValueAsDouble()* 360) % 360);
    //System.out.println(wristMotor.getPosition().getValueAsDouble() - originalWristSensorPosition);
      if (clawPIDCalculation > 0.90){
        clawPIDCalculation = 0.90;
      }
      else if (clawPIDCalculation < -0.90){
        clawPIDCalculation = -0.90;
      }
      // if (Math.abs(clawPIDCalculation)<0.1){
      //   clawPIDCalculation = clawPIDCalculation * 1.032;
      // }
      /*if (Math.abs(clawPIDCalculation)<0.1){
        clawPIDCalculation = clawPIDCalculation * .95;
      }*/
      if (Math.abs(clawPIDCalculation)<0.01){
        clawPIDCalculation = clawPIDCalculation * 5;
        //System.out.println("activated");
      }
      else if (Math.abs(clawPIDCalculation)<0.02){
        clawPIDCalculation = clawPIDCalculation * 1.7;
        //System.out.println("first Activation");
      }
      /*else if (Math.abs(clawPIDCalculation)<0.025){
        clawPIDCalculation = clawPIDCalculation * 1.02;
      }*/
      
      
    wristMotor.set(clawPIDCalculation);

    if (hasAlgae && !driveMotorIsControlled){
      System.out.println("claw setpoint activated");
      clawDriveMotor.set(retainAlgaeController.calculate(getClawDriveMotorPosition(),algaeRetainPosition));
    }
  }
}
