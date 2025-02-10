// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EnumElavatorPosition;
import frc.robot.Constants.ElavatorPositions;

public class ElevatorSubsystem extends SubsystemBase {
  
  /** Elevator Motors */
  private final TalonFX masterMotor;
  private final TalonFX slaveMotor;

  /** Encoder Sensor */
  private final CANcoder elevatorSensor;

  /** PID Controller for precise control */
  private final PIDController elevatorController;

  /** Current Position Tracking */
  private EnumElavatorPosition currentPosition;

  /** Mapping of Enum Positions to Heights */
  private final Map<EnumElavatorPosition, Double> positionMap;

  /** Motion Magic Control */
  private ElevatorPosition targetPosition;
  private double elevatorPIDCalculation;

  /** PIDF Constants */
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.01;
  private static final double kF = 0.05;

  /** Manual Control Mode */
  private boolean manualMode;

  public ElevatorSubsystem() {
    
    /** Initialize position mappings */
    positionMap = new HashMap<EnumElavatorPosition, Double>(){{
      put(EnumElavatorPosition.Rest, ElavatorPositions.Rest);
      put(EnumElavatorPosition.L1, ElavatorPositions.L1);
      put(EnumElavatorPosition.L2, ElavatorPositions.L2);
      put(EnumElavatorPosition.L3, ElavatorPositions.L3);
      put(EnumElavatorPosition.L4, ElavatorPositions.L4);
    }};

    /** Initialize motors */
    masterMotor = new TalonFX(0, "Drivetrain");
    slaveMotor = new TalonFX(0, "Drivetrain");

    /** Configure motors */
    masterMotor.setNeutralMode(NeutralModeValue.Brake);
    slaveMotor.setNeutralMode(NeutralModeValue.Brake);
    slaveMotor.set(masterMotor.get()); // Mirror master motor


    elevatorSensor = new CANcoder(0, "Drivetrain");

    elevatorController = new PIDController(0.1, 0.0, 0.01);  // Tune these values as needed
    elevatorController.setTolerance(0.001);

    /** Initialize Motion Magic Control */
  
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
    SmartDashboard.putString("Current Elevator Position", currentPosition.toString());
    SmartDashboard.putNumber("Elevator Height", getCurrentHeight());

    // Log values for debugging
    System.out.println("Elevator Height: " + getCurrentHeight());
    System.out.println("Manual Mode: " + manualMode);
    // This method will be called once per scheduler run
    elevatorPIDCalculation = elevatorController.calculate(getElevatorSensorPosition(), positionMap.get(targetPosition));
    
    if (!elevatorController.atSetpoint()) {
      masterMotor.set(elevatorPIDCalculation);
    } else {
      masterMotor.set(0);
    }
  }
}
