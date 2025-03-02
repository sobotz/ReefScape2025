// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveConstants{
    public static final double frontLeftDegreeOffset = 132.1875;
    public static final double frontRightDegreeOffset = 164.8828125;
    public static final double backLeftDegreeOffset = 119.619140625;
    public static final double backRightDegreeOffset = 83.759765625;
    public static final double wheelRadius = 0.0508;//in meters;
    public static final double wheelRotationPerMotorRotation = 1/8.14;
  }

  public static class ClawConstants{
    public static final double DEFAULT = -90;
    public static final double HASALGAEDEFAULT = -320;
    public static final double INTAKE = -90;
    public static final double FLOORALGAE = -240;
    public static final double FACINGDOWNREEFALGAE = -270;
    public static final double FACINGUPREEFALGAE = -270;
    public static final double BARGE = -300;
    public static final double L1 = 50;
    public static final double L2 = -236;
    public static final double L3= -236;
    public static final double L4 = -205;
  }
  public static enum ClawPosition{
    DEFAULT,
    INTAKE,
    FLOORALGAE,
    FACINGUPREEFALGAE,
    FACINGDOWNREEFALGAE,
    BARGE,
    L1,
    L2,
    L3,
    L4
  }
  public static enum ElevatorPosition{
    DEFAULT,
    INTAKE,
    FLOORALGAE,
    LOWERALGAE,
    HIGHERALGAE,
    BARGE,
    L1,
    L2,
    L3,
    L4
  }
  public static final class ElevatorConstants {
    //I type PID CONTROLLER
    // public static final double kP = 0.10;//.10
    // public static final double kI = 0.334;//.334
    // public static final double kD = 0.01031;//.01031
    //P type PID Controller
    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0.006;
    public static final double DEFAULT = 10;
    public static final double INTAKE = 20.3;
    public static final double FLOORALGAE = 8;
    public static final double LOWERALGAE = 31;
    public static final double HIGHERALGAE = 43;
    public static final double BARGE = 70;
    public static final double L1 = 15;
    public static final double L2 = 29;
    public static final double L3 = 40;
    public static final double L4 = 70;
  }

  public static class PhotonVisionConstants{
    public static double rightCameraCenterOffset = 50.0;
    public static double leftCameraCenterOffset = -50.0;
    public static String leftCameraName = "4";
    public static String rightCameraName = "3";
  
  }

}
