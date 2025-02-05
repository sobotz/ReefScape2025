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
    public static final double frontLeftDegreeOffset = 312.1875;
    public static final double frontRightDegreeOffset = 7.20703125;
    public static final double backLeftDegreeOffset = 46.0546875;
    public static final double backRightDegreeOffset = 133.505859375;
    public static final double wheelRadius = 0.0508;//in meters;
    public static final double wheelRotationPerMotorRotation = 1/8.14;
  }
}
