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

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
  
  /** Elevator Motors */
  private final TalonFX elevatorMotor;
  private final TalonFX slaveMotor;

  /** Encoder Sensor */
  private final CANcoder elevatorSensor;

  /** PID Controller for precise control */
  private final PIDController elevatorController;

  /** Mapping of Enum Positions to Heights */
  private final Map<ElevatorPosition, Double> positionMap;

  /** Motion Magic Control */

  private double elevatorPIDCalculation;
  private ElevatorPosition targetPosition; 
  /** PIDF Constants */
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.01;

  /** Manual Control Mode */
  private boolean manualMode;

  public ElevatorSubsystem() {
    /** Initialize position mappings */
    
    positionMap = new HashMap<ElevatorPosition, Double>(){{
      put(ElevatorPosition.Rest, ElevatorConstants.Rest);
      put(ElevatorPosition.L1, ElevatorConstants.L1);
      put(ElevatorPosition.L2, ElevatorConstants.L2);
      put(ElevatorPosition.L3, ElevatorConstants.L3);
      put(ElevatorPosition.L4, ElevatorConstants.L4);
    }};

    /** Initialize motors */
    elevatorMotor = new TalonFX(1, "Drivetrain");
    
    slaveMotor = new TalonFX(2, "Drivetrain");

    /** Configure motors */
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    slaveMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorSensor = new CANcoder(3, "Drivetrain");

    elevatorController = new PIDController(kP, kI, kD);  // Tune these values as needed
    elevatorController.setTolerance(0.001);  
  }

  public double getElevatorSensorPosition() {
    return elevatorSensor.getPosition().getValueAsDouble();
  }

  public boolean elevatorAtTargetPosition() {
    return elevatorController.atSetpoint();
  }

  public void setElevatorTargetPosition(ElevatorPosition position) {
    targetPosition = position;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Manual Mode", manualMode);

    System.out.println("Manual Mode: " + manualMode);
    
    elevatorPIDCalculation = elevatorController.calculate(getElevatorSensorPosition(), positionMap.get(targetPosition));
    
    if (!elevatorController.atSetpoint()) {
      elevatorMotor.set(elevatorPIDCalculation);
      slaveMotor.set(elevatorPIDCalculation);
    } else {
      elevatorMotor.set(0);
      slaveMotor.set(0);
    }
  }
}
