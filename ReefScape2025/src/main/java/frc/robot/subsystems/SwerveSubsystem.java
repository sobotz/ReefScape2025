// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
  SwerveDrivePoseEstimator m_odometer;
  ChassisSpeeds chassisSpeed;
  RobotConfig config;
  CurrentLimitsConfigs limitConfigs;
  boolean bargeMode;
  boolean isRedAlliance;
  double previousXError;
  double previousYError;
  double xAtPositionCount;
  double yAtPositionCount;
  double autoTime;
  boolean disableDrive;
  ChassisSpeeds fieldRelativeVelocitySpeeds;
  private PIDController xController;
  private PIDController yController;
  private PIDController headingController;
  PIDController algaeRotationController;
 
  
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
    
    fieldRelativeVelocitySpeeds = new ChassisSpeeds();
    disableDrive = false;
    isRedAlliance = false;
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      isRedAlliance = true;
    }
    else{
      isRedAlliance = false;
    }
    xController = new PIDController(5, 0.0, 0.0);
    yController = new PIDController(5, 0.0, 0.0);
    headingController = new PIDController(4, 0.0, 0.0);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
   
    autoTime = 0;
    
    atTargetPosition = false;
    bargeMode = false;
    xAtPositionCount = 0;
    yAtPositionCount = 0;
    frontLeftDriveMotor = new TalonFX(2,"Drivetrain");
    frontLeftDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    frontLeftTurnMotor = new TalonFX(1,"Drivetrain");
    frontRightDriveMotor = new TalonFX(4,"Drivetrain");
    frontRightDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    frontRightTurnMotor = new TalonFX(3,"Drivetrain");
    backLeftDriveMotor = new TalonFX(8,"Drivetrain");
    backLeftDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    backLeftTurnMotor = new TalonFX(7,"Drivetrain");
    backRightDriveMotor = new TalonFX(6,"Drivetrain");
    backRightDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    backRightTurnMotor = new TalonFX(5,"Drivetrain");

    limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 80;
    limitConfigs.SupplyCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimitEnable = true;
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
    algaeRotationController = new PIDController(0.020,0,0);
    algaeRotationController.enableContinuousInput(0,360); 
    algaeRotationController.setTolerance(0);

    xTranslationController = new PIDController(0.7, 0, 0.0015);//, new TrapezoidProfile.Constraints(1,0.3));
    xTranslationController.setTolerance(0.0);
    yTranslationController =new PIDController(0.7, 0, 0.0015);//, new TrapezoidProfile.Constraints(1,0.3));
    yTranslationController.setTolerance(0.0);

    xVelocityController = new PIDController(0.022,0,0.001);
    //xVelocityController.setTolerance(0.01);
    
    yVelocityController = new PIDController(0.022,0,0.001);
    //yVelocityController.setTolerance(0.01);

    degreeVelocityController = new PIDController(0.009,0.000,0.000);//d0.07
    //degreeVelocityController.setTolerance(0.01);
    targetID = 0;
    once = true;
    robotDegreeOffset = 0;
    driveCommandDisabled = false;
    rotationalVelocityMagnitude =0;
    m_frontLeftLocation = new Translation2d(0.333, 0.27);
    m_frontRightLocation = new Translation2d(0.333, -0.27);
    m_backLeftLocation = new Translation2d(-0.333, 0.27);
    m_backRightLocation = new Translation2d(-0.33, -0.27);

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    
    
    autoRobotDegree = new Rotation2d();
    robotGyro.reset();
    m_odometer = new SwerveDrivePoseEstimator(
      m_kinematics,
      autoRobotDegree,
      new SwerveModulePosition[]{
      frontLeftSwerveModule.getSwerveModulePosition(isRedAlliance),
      frontRightSwerveModule.getSwerveModulePosition(isRedAlliance),
      backLeftSwerveModule.getSwerveModulePosition(isRedAlliance),
      backRightSwerveModule.getSwerveModulePosition(isRedAlliance)},new Pose2d());
    chassisSpeed = new ChassisSpeeds();
    
    
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
    

  //   AutoBuilder.configure(
  //           this::getPose, // Robot pose supplier
  //           this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
  //           this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //           this::velocityControlledDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
  //           new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
  //                   new PIDConstants(5, 0.0, 0.0), // Translation PID constants
  //                   new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
  //           ),
  //           config, // The robot configuration
  //           () -> {
  //             // Boolean supplier that controls when the path will be mirrored for the red alliance
  //             // This will flip the path being followed to the red side of the field.
  //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  //             var alliance = DriverStation.getAlliance();
  //             if (alliance.isPresent()) {
  //               return alliance.get() == DriverStation.Alliance.Red;
  //             }
  //             return false;
  //           },
  //           this // Reference to this subsystem to set requirements
  //   );
  }
  public void setDisableDrive(boolean value){
    disableDrive = value;
  }
  public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();
        //System.out.println(sample.x);
        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        driveFieldRelative(speeds);
    }
  public void driveFieldRelative(ChassisSpeeds speed){
    //System.out.println(speed.vxMetersPerSecond);
    //speed = ChassisSpeeds.fromFieldRelativeSpeeds(speed,new Rotation2d(Math.toRadians(currentRobotDegree)));
    //velocityControlledDrive(speed);
    fieldRelativeVelocitySpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getSpeeds(),new Rotation2d(Math.toRadians(currentRobotDegree)));
    if (isRedAlliance){
      currentRobotDegree = ((currentRobotDegree + 180) % 360);
    }
    
    // Vector xvec = new Vector(getSpeeds().vxMetersPerSecond, currentRobotDegree,true);//ACTUALLY Y FOR ROBOT
    // Vector yvec = new Vector(0,0);
    // yvec = new Vector(getSpeeds().vyMetersPerSecond,(currentRobotDegree + 90) % 360,true);//ACTUTALY X FOR ROBOT
    // Vector cVector = xvec.addVector(yvec);
    // fieldRelativeVelocitySpeeds = new ChassisSpeeds(cVector.getX(),cVector.getY(),speed.omegaRadiansPerSecond);
    
    // double autoDegree = 0;
    // //System.out.println(speed.vyMetersPerSecond);
    xVelocity += xVelocityController.calculate(fieldRelativeVelocitySpeeds.vxMetersPerSecond, speed.vxMetersPerSecond);//ACTUTALY IN AUTO ITS FORWARD IS POSITIVE
    yVelocity += yVelocityController.calculate(fieldRelativeVelocitySpeeds.vyMetersPerSecond, speed.vyMetersPerSecond);//ACTUTALLY IN AUTO LEFT IS POSITIVE
    Vector strafeVector = new Vector(0, 0);
    strafeVector = new Vector(-yVelocity, xVelocity);
    
    rotationalVelocityMagnitude += -degreeVelocityController.calculate(getSpeeds().omegaRadiansPerSecond, speed.omegaRadiansPerSecond);
    // autoDegree = currentRobotDegree;
    // // if (isRedAlliance){
    // //   autoDegree = ((currentRobotDegree + 180) % 360);
    // // }
    
    drive(strafeVector, rotationalVelocityMagnitude,currentRobotDegree, false,false);
  }
  public double getCurrentRobotDegree(){
    return currentRobotDegree;
  }
  public double getAutoRobotDegree(){
    if (isRedAlliance){
      return ((currentRobotDegree + 180) % 360);
    }
    else{
      return currentRobotDegree;
    }
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
      //System.out.println("working");
      drive(strafeVector, rotationalMagnitude, currentRobotDegree, false,false);
    }
  }

  public void velocityControlledDrive(ChassisSpeeds targetSpeeds){
    xVelocity += xVelocityController.calculate(getSpeeds().vxMetersPerSecond, targetSpeeds.vxMetersPerSecond);//ACTUTALY IN AUTO ITS FORWARD IS POSITIVE
    yVelocity += yVelocityController.calculate(getSpeeds().vyMetersPerSecond, targetSpeeds.vyMetersPerSecond);//ACTUTALLY IN AUTO LEFT IS POSITIVE
    Vector strafeVector = new Vector(0,0);
    if (isRedAlliance){
      strafeVector = new Vector(yVelocity, -xVelocity);
    }
    else{
      strafeVector = new Vector(-yVelocity, xVelocity);
    }
    
    rotationalVelocityMagnitude += -degreeVelocityController.calculate(getSpeeds().omegaRadiansPerSecond, targetSpeeds.omegaRadiansPerSecond);
    if (targetSpeeds.vxMetersPerSecond == 0 &&
        targetSpeeds.vyMetersPerSecond == 0 &&
        targetSpeeds.omegaRadiansPerSecond == 0){
      strafeVector = new Vector(0,0);
      rotationalVelocityMagnitude = 0;
    }
    if (isRedAlliance){
      currentRobotDegree = ((currentRobotDegree + 180) % 360);
    }
    drive(strafeVector, rotationalVelocityMagnitude, currentRobotDegree, true,false);
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
      drive(tvec, rotationalMagnitude, (angleOffset + 360) % 360,false,false);
    }else{
      //setDriveCommandDisabled(false);
      drive(new Vector(0, 0),0,currentRobotDegree,false,false);//CHANGE
    }
  }
  public void algaeGrabDrive(double angleOffset, boolean enabled, boolean hasTarget){
    angleOffset = (angleOffset + 360) % 360;
    Vector tv = new Vector(0.15,180,true);
    double setpoint = 0;
    // if (angleOffset>0 && angleOffset < 180){
    //   setpoint = 2;
    // }
    // else if (angleOffset > 180 && angleOffset < 359){
    //   setpoint = 258;
    // }
    if (!hasTarget){
      angleOffset = 0;
      setpoint = 0;
    }
    double rotationalMagnitude =   -algaeRotationController.calculate(angleOffset,setpoint);
    
    
    
    if (enabled){
      drive(tv, rotationalMagnitude, angleOffset,false,false);
    }
    else{
      drive(new Vector(0, 0),0,currentRobotDegree,false,false);
    }
  }

  public void drive(Vector strafeVector, double rotationalMagnitude,double degree, boolean relativeVelocityControlled, boolean disable){
    //System.out.println(strafeVector.getMagnitude());
    if (!disableDrive){
      frontLeftSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,disable);
      frontRightSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,disable);
      backLeftSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,disable);
      backRightSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,disable);
    }
    else{
      System.out.println("Drive disabled");
      frontLeftSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,true);
      frontRightSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,true);
      backLeftSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,true);
      backRightSwerveModule.drive(strafeVector, rotationalMagnitude, degree,relativeVelocityControlled,true);
    }
    
  }

  public Pose2d getPose() {
    return m_odometer.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    SmartDashboard.putNumber("resetPose rotation", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("resetPose x", pose.getX());
    SmartDashboard.putNumber("resetPose y", pose.getY());
    // double rotations = pose.getRotation().getDegrees();
    // rotations -= 180;
    //robotGyro.setYaw(pose.getRotation().getDegrees());
    m_odometer.resetPosition(pose.getRotation(), getAllSwerveModulePositions(), pose);
  }

  public SwerveModulePosition[] getAllSwerveModulePositions(){
    return new SwerveModulePosition[]{
      frontLeftSwerveModule.getSwerveModulePosition(isRedAlliance),
      frontRightSwerveModule.getSwerveModulePosition(isRedAlliance),
      backLeftSwerveModule.getSwerveModulePosition(isRedAlliance),
      backRightSwerveModule.getSwerveModulePosition(isRedAlliance)};
  }

  public SwerveModuleState[] getAllSwerveModuleStates(){
    return new SwerveModuleState[]{
      frontLeftSwerveModule.getSwerveModuleState(isRedAlliance),
      frontRightSwerveModule.getSwerveModuleState(isRedAlliance),
      backLeftSwerveModule.getSwerveModuleState(isRedAlliance),
      backRightSwerveModule.getSwerveModuleState(isRedAlliance)
    };
  }

  public ChassisSpeeds getSpeeds() {
    ChassisSpeeds s = m_kinematics.toChassisSpeeds(getAllSwerveModuleStates());
    if (isRedAlliance){
      s = new ChassisSpeeds(-s.vxMetersPerSecond,-s.vyMetersPerSecond,s.omegaRadiansPerSecond);
    }
    else{
      s = new ChassisSpeeds(s.vxMetersPerSecond,s.vyMetersPerSecond,s.omegaRadiansPerSecond);
    }
    
    //s = new ChassisSpeeds(s.vxMetersPerSecond,s.vyMetersPerSecond,s.omegaRadiansPerSecond);
    
    //return m_kinematics.toChassisSpeeds(getAllSwerveModuleStates());
    return s;

  }
  public void resetCount(){
    xAtPositionCount = 0;
    yAtPositionCount = 0;
  }
  public boolean getAlgaeGrabAtTargetPosition(){
    if ((Math.abs(xTranslationController.getPositionError()) < 0.155)){
      return true;
    }
    else{
      return false;
    }
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
  public Rotation2d getAutoRotation(){
    return autoRobotDegree;
  }
  public void resetGyro(){
    once = true;
  }
  
  public void updateVisionPoseEstimator(Pose2d pose, double time ){
    //System.out.println("UPDATED");
    m_odometer.addVisionMeasurement(pose,time);
    m_odometer.updateWithTime(time, autoRobotDegree, getAllSwerveModulePositions());
  }
  public double getAutoTime(){
    return autoTime;
  }
  @Override
  public void periodic() {
    
    if (once){
      robotGyro.reset();
      robotDegreeOffset = ((((robotGyro.getYaw().getValueAsDouble()) % 360) + 360) % 360);
      once = false;
    }
    currentRobotDegree = ((((robotGyro.getYaw().getValueAsDouble() - robotDegreeOffset) % 360) + 360) % 360);
    ChassisSpeeds speed = getSpeeds();
    chassisSpeed = speed;
    if (currentRobotDegree > 180){
      autoRobotDegree = new Rotation2d((currentRobotDegree - 360) * (Math.PI/180));
    }
    else{
      autoRobotDegree = new Rotation2d(currentRobotDegree * (Math.PI/180));
    }
    
    if (isRedAlliance){
      autoRobotDegree = new Rotation2d(Math.toRadians(autoRobotDegree.getDegrees() + 180));
      //System.out.println(autoRobotDegree);
    }
    
    
    //Set the current degree of the robot 
    
    
    autoTime = Timer.getFPGATimestamp();
    m_odometer.update(autoRobotDegree,new SwerveModulePosition[]{
      frontLeftSwerveModule.getSwerveModulePosition(isRedAlliance),
      frontRightSwerveModule.getSwerveModulePosition(isRedAlliance),
      backLeftSwerveModule.getSwerveModulePosition(isRedAlliance),
      backRightSwerveModule.getSwerveModulePosition(isRedAlliance)}
    );
    SmartDashboard.putBoolean("redAlliance",isRedAlliance);
    SmartDashboard.putNumber("autoRobotDegree", autoRobotDegree.getDegrees());
    SmartDashboard.putNumber("odometerDegree", m_odometer.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("x Trans", m_odometer.getEstimatedPosition().getX());
    SmartDashboard.putNumber("y Trans", m_odometer.getEstimatedPosition().getY());
    SmartDashboard.putNumber("x velocity", getSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("y velocity", getSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("theta velocity", Math.toDegrees(getSpeeds().omegaRadiansPerSecond));
    SmartDashboard.putNumber("FR x velocity", fieldRelativeVelocitySpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("FR y velocity", fieldRelativeVelocitySpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("FR theta velocity", fieldRelativeVelocitySpeeds.omegaRadiansPerSecond);
    //SmartDashboard.putNumber("x Vel", getSpeeds().vxMetersPerSecond);
    //SmartDashboard.putNumber("y Vel", getSpeeds().vyMetersPerSecond);
    //SmartDashboard.putNumber("raw drive sensor", frontLeftSwerveModule.getRawDriveSensorPosition());
    //SmartDashboard.putNumber("Angular Velocity",robotGyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180));
    //SmartDashboard.putNumber("degree",currentRobotDegree);
    //SmartDashboard.putNumber("xOffsetDrive", xTranslationController.getPositionError());
    //SmartDashboard.putNumber("yOffsetDrive", yTranslationController.getPositionError());
    //System.out.println(isRedAlliance);
    
  }
}
