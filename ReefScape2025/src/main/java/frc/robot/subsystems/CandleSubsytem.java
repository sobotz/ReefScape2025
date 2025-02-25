// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSubsytem extends SubsystemBase {
  /** Creates a new CandleSubsytem. */
  CANdle candle;
  boolean candleInUse;
  public CandleSubsytem() {
    candle = new CANdle(0);
    candleInUse = false;
  }
  public void setCandle(int R,int G,int B,double brightness, boolean determiningBoolean){
    if (determiningBoolean){
      candleInUse = true;
      candle.configBrightnessScalar(brightness);
      candle.setLEDs(R,G,B);
    }else{
      candleInUse = false; 
    }
  }
  @Override
  public void periodic() {
    if(!candleInUse){
      candle.configBrightnessScalar(0);
      candle.setLEDs(0,0,0);
    }
    // This method will be called once per scheduler run
  }
}
