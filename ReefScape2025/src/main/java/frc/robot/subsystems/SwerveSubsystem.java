// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.json.simple.parser.Yytoken;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  //FRONT LEFT MOTORS
  TalonFX frontLeftDriveMotor;
  TalonFX frontLeftTurnMotor;
  //FRONT RIGHT MOTORS
  TalonFX frontRightDriveMotor;
  TalonFX frontRightTurnMotor;
  //BACK LEFT MOTORS
  TalonFX backLeftDriveMotor;
  TalonFX backLeftTurnMotor;
  //BACK RIGHT MOTORS
  TalonFX backRightDriveMotor;
  TalonFX backRightTurnMotor;
  //SWERVE MODULE CAN ENCODERS 
  CANcoder frontLeftEncoder;
  CANcoder frontRightEncoder;
  CANcoder backLeftEncoder;
  CANcoder backRightEncoder;
  //GYROSCOPE PIGEON 
  Pigeon2 robotGyro;
  //SWERVE MODULES 
  SwerveModule frontLeftSwerveModule;
  SwerveModule frontRightSwerveModule;
  SwerveModule backLeftSwerveModule;
  SwerveModule backRightSwerveModule;
  //PID CONTROLLERS
  PIDController rotationController;
  PIDController angleCorrectionController;
  PIDController xTranslationController;
  PIDController yTranslationController;

  PIDController xVelocityController;
  PIDController yVelocityController;
  PIDController degreeVelocityController;

  //CURRENT STATUS VARIABLES
  double currentRobotDegree;
  double robotDegreeOffset;
  double xVelocity;
  double yVelocity;
  double xTranslation;
  double yTranslation;
  double rotationalVelocityMagnitude;
  int targetID;

  boolean angleCorrectionMode;
  boolean once;
  boolean driveCommandDisabled;
  boolean atTargetPosition;

  Translation2d m_frontLeftLocation;
  Translation2d m_frontRightLocation;
  Translation2d m_backLeftLocation;
  Translation2d m_backRightLocation;
  Rotation2d autoRobotDegree;

  SwerveDriveKinematics m_kinematics;
  SwerveDriveOdometry m_odometer;
  ChassisSpeeds chassisSpeed;
  RobotConfig config;
  CurrentLimitsConfigs limitConfigs;
  boolean bargeMode;
  boolean isRedAlliance;
  double previousXError;
  double previousYError;
  double xAtPositionCount;
  double yAtPositionCount;
  /*Update requirements
   * *******SWERVE SUBSYSTEM*******
   * ID every device -- DONE
   * xTranslational PID controller needs tuning  
   * yTranslational PID controller needs tuning
   * xVelocity PID controller needs tuning
   * yVelocity PID controller needs tuning
   * rotationalVelocity PID controller needs tuning -- DONE
   * drivetrain motors may need to be set to the correct canbus -- DONE
   * RotationalValueDegree inputted as a parameter into the swerve modules needs to be updated (robot is not square) -- DONE
   * ------------------------------
   * *******SWERVE MODULE*******
   * getPosition for drive motor needs to be checked -- DONE
   * getVelocity for drive motor needs to be checked -- DONE
   * degree PID Controller needs to be tune -- DONE
   * ------------------------------
   * *******PATHPLANNER GUI*******
   * robot stats needs to be updated
   * all robot limits needs to be tested and implements(maximize speed without losing accuracy)
   */


  public SwerveSubsystem() {
    //SWERVE MOTORS INSTANTIATION
    isRedAlliance = false;
    atTargetPosition = false;
    bargeMode = false;
    xAtPositionCount = 0;
    yAtPositionCount = 0;
    frontLeftDriveMotor = new TalonFX(2,"Drivetrain");
    frontLeftTurnMotor = new TalonFX(1,"Drivetrain");
    frontRightDriveMotor = new TalonFX(4,"Drivetrain");
    frontRightTurnMotor = new TalonFX(3,"Drivetrain");
    backLeftDriveMotor = new TalonFX(8,"Drivetrain");
    backLeftTurnMotor = new TalonFX(7,"Drivetrain");
    backRightDriveMotor = new TalonFX(6,"Drivetrain");
    backRightTurnMotor = new TalonFX(5,"Drivetrain");

    limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 50;
    limitConfigs.StatorCurrentLimitEnable = true;
    frontLeftDriveMotor.getConfigurator().apply(limitConfigs);
    frontLeftTurnMotor.getConfigurator().apply(limitConfigs);
    frontRightDriveMotor.getConfigurator().apply(limitConfigs);
    frontRightTurnMotor.getConfigurator().apply(limitConfigs);
    backLeftDriveMotor.getConfigurator().apply(limitConfigs);
    backLeftTurnMotor.getConfigurator().apply(limitConfigs);
    backRightDriveMotor.getConfigurator().apply(limitConfigs);
    backRightTurnMotor.getConfigurator().apply(limitConfigs);

    //ENCODER INSTANTIATION
    frontLeftEncoder = new CANcoder(9,"Drivetrain");
    frontRightEncoder = new CANcoder(10,"Drivetrain");
    backLeftEncoder = new CANcoder(12,"Drivetrain" );
    backRightEncoder = new CANcoder(11,"Drivetrain");
    //PIGEON2 INSTANTIATION
    robotGyro = new Pigeon2(0,"Drivetrain");
    //SWERVE MODULE INSTANTIATION
    frontLeftSwerveModule = new SwerveModule(frontLeftDriveMotor,frontLeftTurnMotor,frontLeftEncoder,310.07289005,Constants.SwerveConstants.frontLeftDegreeOffset);
    frontRightSwerveModule = new SwerveModule(frontRightDriveMotor,frontRightTurnMotor,frontRightEncoder,229.92710995,Constants.SwerveConstants.frontRightDegreeOffset);
    backLeftSwerveModule = new SwerveModule(backLeftDriveMotor,backLeftTurnMotor,backLeftEncoder,49.92710995,Constants.SwerveConstants.backLeftDegreeOffset);
    backRightSwerveModule = new SwerveModule(backRightDriveMotor,backRightTurnMotor,backRightEncoder,130.07289005,Constants.SwerveConstants.backRightDegreeOffset);
    //PID CONTROLLER INSTANTIATION
    angleCorrectionController = new PIDController(0,0,0);
    //Connects 0 degrees to 360 degrees to allow for the least distance error from current to target
    angleCorrectionController.enableContinuousInput(0,360);
    //Allows for leeway if the current degree is not exactly on target
    angleCorrectionController.setTolerance(0.2);

    rotationController = new PIDController(0.006,0,0);
    rotationController.enableContinuousInput(0,360); 
    rotationController.setTolerance(0);

    xTranslationController = new PIDController(0.74, 0, 0.0015);//, new TrapezoidProfile.Constraints(1,0.3));
    xTranslationController.setTolerance(0.0);
    yTranslationController =new PIDController(0.74, 0, 0.0015);//, new TrapezoidProfile.Constraints(1,0.3));
    yTranslationController.setTolerance(0.0);

    xVelocityController = new PIDController(0.022,0,0.001);
    //xVelocityController.setTolerance(0.01);
    
    yVelocityController = new PIDController(0.022,0,0.001);
    //yVelocityController.setTolerance(0.01);

    degreeVelocityController = new PIDController(0.011,0.000,0.000);//d0.07
    //degreeVelocityController.setTolerance(0.01);
    targetID = 0;
    once = true;
    robotDegreeOffset = 0;
    driveCommandDisabled = false;
    rotationalVelocityMagnitude =0;
    m_frontLeftLocation = new Translation2d(0.355, 0.355);
    m_frontRightLocation = new Translation2d(0.355, -0.355);
    m_backLeftLocation = new Translation2d(-0.355, 0.355);
    m_backRightLocation = new Translation2d(-0.355, -0.355);

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    autoRobotDegree = new Rotation2d();
    robotGyro.reset();
    m_odometer = new SwerveDriveOdometry(
      m_kinematics,
      autoRobotDegree,
      new SwerveModulePosition[]{
      frontLeftSwerveModule.getSwerveModulePosition(),
      frontRightSwerveModule.getSwerveModulePosition(),
      backLeftSwerveModule.getSwerveModulePosition(),
      backRightSwerveModule.getSwerveModulePosition()}
    );
    chassisSpeed = new ChassisSpeeds();
    
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      isRedAlliance = true;
    }
    else{
      isRedAlliance = false;
    }
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    
    
    
    //isRedAlliance = true;
    //isRedAlliance = false;//CHANGEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

    previousXError = 0;
    previousYError = 0;
    
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::velocityControlledDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
  public double getCurrentRobotDegree(){
    return currentRobotDegree;
  }
  public void setBargeMode(boolean bargeMode){
    this.bargeMode = bargeMode;
  }

  public void setDriveCommandDisabled(boolean mode){
    driveCommandDisabled = mode;
  }
  public int getTargetID(){
    return targetID;
  }
  public void setTargetID(int id){
    targetID = id;
  }
  public boolean getIsRedAlliance(){
    return isRedAlliance;
  }

  public void driverControlledDrive(Vector strafeVector, Vector rotationVector){
    if (!driveCommandDisabled){
      double rotationalMagnitude = -rotationController.calculate(currentRobotDegree,rotationVector.getDegrees());
      if (Math.abs(rotationalMagnitude) < 0.01){
        rotationalMagnitude = 0;
      }
      if (bargeMode){
        strafeVector.setMagnitude(strafeVector.getMagnitude() * .1);
        //rotationalMagnitude = -rotationController.calculate(currentRobotDegree,180);
      }
      drive(strafeVector, rotationalMagnitude, currentRobotDegree, false);
    }
  }

  public void velocityControlledDrive(ChassisSpeeds targetSpeeds){
    xVelocity += xVelocityController.calculate(getSpeeds().vxMetersPerSecond, targetSpeeds.vxMetersPerSecond);//ACTUTALY IN AUTO ITS FORWARD IS POSITIVE
    yVelocity += yVelocityController.calculate(getSpeeds().vyMetersPerSecond, targetSpeeds.vyMetersPerSecond);//ACTUTALLY IN AUTO LEFT IS POSITIVE
    
    Vector strafeVector = new Vector(-yVelocity, xVelocity);
    rotationalVelocityMagnitude += -degreeVelocityController.calculate(getSpeeds().omegaRadiansPerSecond, targetSpeeds.omegaRadiansPerSecond);
    if (targetSpeeds.vxMetersPerSecond == 0 &&
        targetSpeeds.vyMetersPerSecond == 0 && 
        targetSpeeds.omegaRadiansPerSecond == 0){
      strafeVector = new Vector(0,0);
      rotationalVelocityMagnitude = 0;
    }
    drive(strafeVector, rotationalVelocityMagnitude, currentRobotDegree, true);
  }

  public void reefControlledDrive(double xOffset, double yOffset,double angleOffset, double xTarget, double yTarget,boolean enabled){
    //System.out.println("xoffset: " + xOffset);
    //System.out.println("yoffset: " + yOffset);
    //System.out.println("yTarget: " + yTarget);
    //System.out.println("driveCommand: " + driveCommandDisabled);
    //System.out.println("xTarget: " + xTarget);
    //System.out.println("yTarget: " +  yTarget);
    angleOffset = (angleOffset + 360) % 360;
    double xCalculation = -xTranslationController.calculate( -xOffset,xTarget);
    double yCalculation = yTranslationController.calculate(yOffset,yTarget);
    if (Math.abs(xTranslationController.getPositionError()) > 0.02){
      xCalculation = -xTranslationController.calculate( -xOffset,xTarget);
    }
    else{
      xCalculation = 0;
    }
    if (Math.abs(yTranslationController.getPositionError())> 0.02){
      yCalculation = yTranslationController.calculate(yOffset,yTarget);
      //System.out.println(yTranslationController.getPositionError());
    }
    else{
      yCalculation = 0;
    }
    Vector tvec = new Vector(xCalculation, yCalculation);
    Vector rvec = new Vector(1, 0 , true);
    //angleRotationController.
    //System.out.println(rvec.getMagnitude());
    double rotationalMagnitude = -rotationController.calculate(angleOffset,rvec.getDegrees());
    
    //rotationalMagnitude = 0;
    //System.out.println("trans vec mag :  " + tvec.getMagnitude());
    //System.out.println("trans vec degree: " + tvec.getDegrees());
    
    if (enabled){ 
      drive(tvec, rotationalMagnitude, (angleOffset + 360) % 360,false);
    }else{
      //setDriveCommandDisabled(false);
      drive(new Vector(0, 0),0,currentRobotDegree,false);//CHANGE
    }
  }

  public void drive(Vector strafeVector, double rotationalMagnitude,double currentRobotDegree, boolean relativeVelocityControlled){
    //System.out.println(strafeVector.getMagnitude());
    frontLeftSwerveModule.drive(strafeVector, rotationalMagnitude, currentRobotDegree,relativeVelocityControlled);
    frontRightSwerveModule.drive(strafeVector, rotationalMagnitude, currentRobotDegree,relativeVelocityControlled);
    backLeftSwerveModule.drive(strafeVector, rotationalMagnitude, currentRobotDegree,relativeVelocityControlled);
    backRightSwerveModule.drive(strafeVector, rotationalMagnitude, currentRobotDegree,relativeVelocityControlled);
  }

  public Pose2d getPose() {
    return m_odometer.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_odometer.resetPosition(autoRobotDegree, getAllSwerveModulePositions(), pose);
  }

  public SwerveModulePosition[] getAllSwerveModulePositions(){
    return new SwerveModulePosition[]{
      frontLeftSwerveModule.getSwerveModulePosition(),
      frontRightSwerveModule.getSwerveModulePosition(),
      backLeftSwerveModule.getSwerveModulePosition(),
      backRightSwerveModule.getSwerveModulePosition()};
  }

  public SwerveModuleState[] getAllSwerveModuleStates(){
    return new SwerveModuleState[]{
      frontLeftSwerveModule.getSwerveModuleState(),
      frontRightSwerveModule.getSwerveModuleState(),
      backLeftSwerveModule.getSwerveModuleState(),
      backRightSwerveModule.getSwerveModuleState()
    };
  }

  public ChassisSpeeds getSpeeds() {
    return m_kinematics.toChassisSpeeds(getAllSwerveModuleStates());
  }
  public void resetCount(){
    xAtPositionCount = 0;
    yAtPositionCount = 0;
  }
  public boolean getAtTargetPosition(boolean hasTarget){
    boolean xAtTarget = false;
    boolean yAtTarget = false;
    
    if ((Math.abs(xTranslationController.getPositionError())<0.04)){
      
      if (Math.abs(previousXError - xTranslationController.getPositionError()) <0.03){//(Math.abs(clawController.getError())<0.13) && Math.abs(clawPIDCalculation)<0.0023){
        xAtPositionCount += 1;
        //System.out.println("xCount: " + xAtPositionCount);
      }  
      else{
        xAtPositionCount = 0;
      }
      //SwerveDrivePoseEstimator example = new SwerveDrivePoseEstimator(m_kinematics, autoRobotDegree, getAllSwerveModulePositions(), getPose());
      
      previousXError = xTranslationController.getPositionError();
      if (xAtPositionCount > 1){
        //System.out.println("atPositionCLaw");
        xAtTarget = true;
      }
      else{
        xAtTarget = false;
      }
    }
    else{
      previousXError = xTranslationController.getPositionError();
      xAtTarget = false;
    }


    if ((Math.abs(yTranslationController.getPositionError())<0.04)){
      //System.out.println("inrange");
      if (Math.abs(previousYError - yTranslationController.getPositionError()) <0.03){//(Math.abs(clawController.getError())<0.13) && Math.abs(clawPIDCalculation)<0.0023){
        yAtPositionCount += 1;
        //System.out.println("yCount: " + yAtPositionCount);
      }  
      else{
        yAtPositionCount = 0;
      }

      previousYError = yTranslationController.getPositionError();
      if (yAtPositionCount > 1){
        System.out.println();
        yAtTarget = true;
      }
      else{
        yAtTarget = false;
      }
    }
    else{
      previousYError = yTranslationController.getPositionError();
      yAtTarget = false;
      
    }
    if ((yAtTarget && xAtTarget)){
      //System.out.println("AtMovementPoint");
      return true;
    }
    else{
      //System.out.println("CHECKING");
      return false;
    }
  }
  public void resetGyro(){
    once = true;
  }
  

  @Override
  public void periodic() {
    
    ChassisSpeeds speed = getSpeeds();
    chassisSpeed = speed;
    if (currentRobotDegree > 180){
      autoRobotDegree = new Rotation2d((currentRobotDegree - 360) * (Math.PI/180));
    }
    else{
      autoRobotDegree = new Rotation2d(currentRobotDegree * (Math.PI/180));
    }
    //Set the current degree of the robot 
    if (once){
      robotGyro.reset();
      robotDegreeOffset = ((((robotGyro.getYaw().getValueAsDouble()) % 360) + 360) % 360);
      once = false;
    }
    currentRobotDegree = ((((robotGyro.getYaw().getValueAsDouble() - robotDegreeOffset) % 360) + 360) % 360);
    m_odometer.update(autoRobotDegree,new SwerveModulePosition[]{
      frontLeftSwerveModule.getSwerveModulePosition(),
      frontRightSwerveModule.getSwerveModulePosition(),
      backLeftSwerveModule.getSwerveModulePosition(),
      backRightSwerveModule.getSwerveModulePosition()}
    );
    SmartDashboard.putNumber("x Trans", m_odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("y Trans", m_odometer.getPoseMeters().getY());
    SmartDashboard.putNumber("x Vel", getSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("y Vel", getSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("raw drive sensor", frontLeftSwerveModule.getRawDriveSensorPosition());
    SmartDashboard.putNumber("Angular Velocity",robotGyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180));
    SmartDashboard.putNumber("degree",currentRobotDegree);
    SmartDashboard.putNumber("xOffsetDrive", xTranslationController.getPositionError());
    SmartDashboard.putNumber("yOffsetDrive", yTranslationController.getPositionError());
    //System.out.println(isRedAlliance);
    
  }
}
