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
    public static final double frontLeftDegreeOffset = 128.1875;
    public static final double frontRightDegreeOffset = 275.8828125;
    public static final double backLeftDegreeOffset = 118.619140625;
    public static final double backRightDegreeOffset = 52.759765625;
    public static final double wheelRadius = 0.0508;//in meters;
    public static final double wheelRotationPerMotorRotation = 1/8.14;
  }

  public static class ClawConstants{
    public static final double DEFAULT = -87;
    public static final double HASALGAEDEFAULT = -360;
    public static final double HASALGAEDEFAULT2 = 0;
    public static final double INTAKE = -87;
    public static final double FLOORALGAE = -220.0137;
    public static final double PROCESSOR = -270;
    public static final double PROCESSOR2 = 90;
    public static final double FACINGDOWNREEFALGAE = -240;
    public static final double FACINGUPREEFALGAE = -300;
    public static final double REVERSEFACINGUPREEFALGAE = 63.5;
    public static final double BARGE = -326;
    public static final double BARGE2 = 34;
    public static final double L1 = 60;
    public static final double L2 = -239;
    public static final double L3= -239;
    public static final double L4 = -228.7;
    public static final double TEMPPOSITION = -180;
  }
  public static enum ClawPosition{
    DEFAULT,
    INTAKE,
    FLOORALGAE,
    PROCESSOR,
    PROCESSOR2,
    FACINGUPREEFALGAE,
    FACINGDOWNREEFALGAE,
    REVERSEFACINGUPALGAE,
    BARGE,
    BARGE2,
    L1,
    L2,
    L3,
    L4,
    TEMPPOSITION
  }
  public static enum ElevatorPosition{
    DEFAULT,
    ALGAETEMP,
    INTAKE,
    CLIMB,
    FLOORALGAE,
    PROCESSOR,
    LOWESTALGAE,
    LOWERALGAE,
    MIDALGAE,
    HIGHERALGAE,
    BARGE,
    L1,
    L2,
    L3,
    L4,
    TEMPPOSITION
  }
  public static final class ElevatorConstants {
    //I type PID CONTROLLER
    // public static final double kP = 0.10;//.10
    // public static final double kI = 0.334;//.334
    // public static final double kD = 0.01031;//.01031
    //P type PID Controller
    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0.000;//0.006
    public static final double kG = 0.6;
    public static final double kS = 0.12;
    public static final double kV = 0.13;//0.23
    public static final double kA = 0.003;

    public static final double DEFAULT = 20;//6
    public static final double ALGAETEMP = 12;
    public static final double INTAKE = 16.3;
    public static final double FLOORALGAE = 7.3;
    public static final double PROCESSOR = 6;
    public static final double CLIMB = 0.5;
    public static final double LOWESTALGAE = 21.6;
    public static final double LOWERALGAE = 30;
    public static final double MIDALGAE = 36;
    public static final double HIGHERALGAE = 46;
    public static final double BARGE = 70;
    public static final double L1 = 5;
    public static final double L2 = 28.4;
    public static final double L3 = 44;
    public static final double L4 = 71.5;
    public static final double TEMPPOSITION = 30;
  }

  public static class PhotonVisionConstants{
    public static double m4CameraCenterOffset = 50.0;
    public static double m3CameraCenterOffset = -50.0;
    public static String m3CameraName = "Module_3_Arducam_OV2311";
    public static String m4CameraName = "Module_4_Arducam_OV2311";
  
  }

}
