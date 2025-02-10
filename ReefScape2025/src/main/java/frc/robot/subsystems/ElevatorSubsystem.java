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
  private final CANcoder sensor;

  /** PID Controller for precise control */
  private final PIDController pidController;

  /** Current Position Tracking */
  private EnumElavatorPosition currentPosition;

  /** Mapping of Enum Positions to Heights */
  private final Map<EnumElavatorPosition, Double> positionMap;

  /** Motion Magic Control */
  private final MotionMagicVoltage motionControl;

  /** PIDF Constants */
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.01;
  private static final double kF = 0.05;

  /** Elevator Motion Limits */
  private static final double MIN_HEIGHT = 0.0;    
  private static final double MAX_HEIGHT = 100.0;  

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

    /** Initialize sensor */
    sensor = new CANcoder(0, "Drivetrain");

    /** Initialize Motion Magic Control */
    motionControl = new MotionMagicVoltage(0);

    /** Initialize PID Controller */
    pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(0.5);

    /** Set default position */
    currentPosition = EnumElavatorPosition.Rest;

    /** Default to manual control mode */
    manualMode = false;
  }

  /**
   * Toggle manual mode for direct motor control
   */
  public void toggleManual() {
    manualMode = !manualMode;
  }

  /**
   * Move the elevator to a predefined position using Motion Magic
   * @param position The desired EnumElavatorPosition
   */
  public void moveToPosition(EnumElavatorPosition position) {
    if (positionMap.containsKey(position)) {
      double targetHeight = positionMap.get(position);
      targetHeight = Math.max(MIN_HEIGHT, Math.min(targetHeight, MAX_HEIGHT));

      motionControl.withPosition(targetHeight);
      masterMotor.setControl(motionControl);

      currentPosition = position;
    } else {
      System.err.println("Invalid elevator position requested: " + position);
    }
  }

  /**
   * Manually control elevator with direct speed input
   * @param speed The speed of the elevator (-1 to 1)
   */
  public void manualMove(double speed) {
    if (manualMode) {
      if (getCurrentHeight() <= MIN_HEIGHT && speed < 0) {
        stop();
      } else if (getCurrentHeight() >= MAX_HEIGHT && speed > 0) {
        stop();
      } else {
        masterMotor.set(speed);
        slaveMotor.set(speed);
      }
    }
  }

  /**
   * Stops the elevator
   */
  public void stop() {
    masterMotor.set(0);
    slaveMotor.set(0);
  }

  /**
   * Get the current elevator position in inches
   * @return Current height
   */
  public double getCurrentHeight() {
    return sensor.getPosition().getValueAsDouble();
  }

  /**
   * Get the current set position
   * @return EnumElavatorPosition
   */
  public EnumElavatorPosition getCurrentPosition() {
    return currentPosition;
  }

  @Override
  public void periodic() {
    // Update SmartDashboard for debugging
    SmartDashboard.putBoolean("Manual Mode", manualMode);
    SmartDashboard.putString("Current Elevator Position", currentPosition.toString());
    SmartDashboard.putNumber("Elevator Height", getCurrentHeight());

    // Log values for debugging
    System.out.println("Elevator Height: " + getCurrentHeight());
    System.out.println("Manual Mode: " + manualMode);
  }
}
