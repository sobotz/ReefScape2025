// Copym4 (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class PhotonVisionSubsystem extends SubsystemBase {
  //SwereSubsystem
  SwerveSubsystem m_swerveSubsystem;
  //Cameras
   PhotonCamera m3Camera;
   PhotonCamera m4Camera;
   

   PhotonPoseEstimator m3Pose;
   PhotonPoseEstimator m4Pose;
   Transform3d m3PoseTransform3d;
   Transform3d m4PoseTranform3d;
   AprilTagFieldLayout aprilTags;
   List<Pose2d> estimates;
   List<Pose3d> targets;
   PoseStrategy kStrategy;
   
   //Camera Data
   PhotonPipelineResult m3CameraResult;
   PhotonPipelineResult m4CameraResult;
   

   Transform3d m3CameraTargetInfo;
   Transform3d m4CameraTargetInfo;
   

   double cameraXOffset;
   double cameraYOffset;
   double cameraAngleOffset3D;
   double cameraAngleOffset2D;

   double robotXOffset;
   double robotYOffset;
   double robotAngleOffset;
  
   double[] m3TargetData;
   double[] m4TargetData;
   

   List<PhotonTrackedTarget> m3CameraTargets;
   List<PhotonTrackedTarget> m4CameraTargets;
   

   PhotonTrackedTarget m3CameraCurrentTarget;
   PhotonTrackedTarget m4CameraCurrentTarget;
   

   double tempXOffset;
   double tempYOffset;
   double tempAngleOffset3D;
   double tempAngleOffset2D;
   double tempHasTarget;

   boolean hasTarget;


   int m3CameraTargetId;
   int m4CameraTargetId;
   int id;
   boolean usingM3Camera;

   double xTarget;
   double yTarget;

   boolean atTargetPosition;
   boolean alignActive;


   boolean aReef;
   boolean bReef;
   boolean cReef;
   boolean dReef;
   boolean eReef;
   boolean fReef;
   boolean gReef;
   boolean hReef;
   boolean iReef;
   boolean jReef;
   boolean kReef;
   boolean lReef;

   double reefNumber;
   boolean emergencyReset;
   EstimatedRobotPose m3RobotPose;
   EstimatedRobotPose m4RobotPose;


  public PhotonVisionSubsystem(SwerveSubsystem subsystem) {
    targets = new ArrayList<>();
    estimates = new ArrayList<>();
    aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    kStrategy = PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;
    //kStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    m3PoseTransform3d = new Transform3d(-0.225425, -0.2794, 0.20955, new Rotation3d(0, -0.436332,2.44346));//RADIANS//CHANGE Z HEIGHT2.44346//2.26893
    m4PoseTranform3d = new Transform3d(-0.2225425, 0.2794, 0.20955, new Rotation3d(0, -0.436332,-2.44346));//RADIANS//CHANGE Z HEIGHT
    emergencyReset = false;
    reefNumber = 0;
    aReef = false;
    bReef = false;
    cReef = false;
    dReef = false;
    eReef = false;
    fReef = false;
    gReef = false;
    hReef = false;
    iReef = false;
    jReef = false;
    kReef = false;
    lReef = false;
    atTargetPosition = false;
    m_swerveSubsystem = subsystem;
    xTarget = 0;
    yTarget = 0;
    alignActive = false;
    m3Camera = new PhotonCamera(Constants.PhotonVisionConstants.m3CameraName);
    m4Camera = new PhotonCamera(Constants.PhotonVisionConstants.m4CameraName);
  
    id = 20;
    cameraXOffset = 0;
    cameraYOffset = 0;
    cameraAngleOffset3D = 0;
    cameraAngleOffset2D = 0;

    tempXOffset = 0;
    tempYOffset = 0;
    tempAngleOffset3D = 0;
    tempAngleOffset2D = 0;

    robotXOffset = 0;
    robotYOffset = 0;
    robotAngleOffset = 0;


    usingM3Camera = false;
    xTarget = 0;
    yTarget = 1;
    m3Pose = new PhotonPoseEstimator(aprilTags, kStrategy, m3PoseTransform3d);
    m4Pose = new PhotonPoseEstimator(aprilTags, kStrategy, m4PoseTranform3d);
    
    m3CameraResult = m3Camera.getLatestResult();
    m4CameraResult = m4Camera.getLatestResult();
    m3TargetData = getData(m3CameraResult);
    m4TargetData = getData(m4CameraResult);
    
  
  }
  /*public void lightUpReef(int id, boolean isRightSide){
    if (id>=6 && id<=11){
      reefNumber = 
    }
  }*/
  public void enableEmergencyReset(){
    emergencyReset = true;
  }
  public void disableEmergencyReset(){
    emergencyReset = false;
  }
  public boolean getEmergencyReset(){
    return emergencyReset;
  }
  
  public void enableAlign(boolean enableAlign,double x, double y, int id){
    xTarget = x;
    yTarget = y;
    alignActive = enableAlign;
    m_swerveSubsystem.setTargetID(id);
    this.id = id;
    if (!enableAlign){
      setDriveCommandDisabled(false);
    }
  }

  
  public double[] getData(PhotonPipelineResult result){
    if(result.hasTargets() && result.getTargets().stream().anyMatch(t -> Arrays.asList(id).contains(t.getFiducialId()))){
      // get all targets
      //System.out.println("working");
      result.getTargets().stream()
        // filter out all except id
        .filter(t -> Arrays.asList(id).contains(t.getFiducialId()))
        // set offsets of id
        .forEach(target -> {
          tempXOffset = target.getBestCameraToTarget().getX();//distance to target front back
          tempYOffset = -target.getBestCameraToTarget().getY();//distance to target left right
          tempAngleOffset3D = target.getBestCameraToTarget().getRotation().getZ();
          if (tempAngleOffset3D * (180/Math.PI)<0){
            tempAngleOffset3D = (-1)*(180+tempAngleOffset3D*(180/Math.PI));
          }else{
            tempAngleOffset3D = (180-tempAngleOffset3D*(180/Math.PI));
          }
          //System.out.println(tempAngleOffset3D);
          tempAngleOffset2D = target.getYaw();
          //System.out.println(target.getYaw());
          //CHECK IF GET ANGLE WORKS AS INTENDED
          tempHasTarget = 1;});
    }
    else{
      tempXOffset = 0;
      tempYOffset = 0;
      tempAngleOffset3D = 0;
      tempAngleOffset2D = 0;
      tempHasTarget = 0;
    }
    //System.out.println(tempAngleOffset2D);
    double[] returnList ={tempXOffset,tempYOffset,tempAngleOffset3D,tempAngleOffset2D, tempHasTarget};
    return returnList; 
  }
  public boolean getIsRedAlliance(){
    return m_swerveSubsystem.getIsRedAlliance();
  }
  public void align(double x, double y,int ID,boolean enabled){
    m_swerveSubsystem.setTargetID(ID);
    //System.out.println("running");
    //System.out.println(m3TargetData[4]);

      // robotYOffset = z * Math.cos((Math.PI/180) * (180-(cameraAngleOffset2D + 50)));
      // robotXOffset = z * Math.sin((Math.PI/180) * (180-(cameraAngleOffset2D + 50))) - 0.28;
      
    
    //System.out.println(hasTarget);
    if (m3TargetData[4] == 1 && m4TargetData[4] == 1){
      if (Math.abs(m3TargetData[3])<Math.abs(m4TargetData[3])){
        //System.out.println("1st");
        hasTarget = true;
        usingM3Camera = true;
        cameraXOffset = m3TargetData[0];
        cameraYOffset = m3TargetData[1];
        cameraAngleOffset3D = m3TargetData[2];
        cameraAngleOffset2D = m3TargetData[3];
      }
      else if (Math.abs(m3TargetData[3])>Math.abs(m4TargetData[3])){
        //System.out.println("2st");
        hasTarget = true;
        usingM3Camera = false;
        cameraXOffset = m4TargetData[0];
        cameraYOffset = m4TargetData[1];
        cameraAngleOffset3D = m4TargetData[2];
        cameraAngleOffset2D = m4TargetData[3];
      }
      else if ((Math.abs(m3TargetData[3]) == Math.abs(m4TargetData[3])) && m3TargetData[3] != 0){
        //System.out.println("3st");
        hasTarget = true;
        usingM3Camera = true;
        cameraXOffset = m3TargetData[0];
        cameraYOffset = m3TargetData[1];
        cameraAngleOffset3D = m3TargetData[2];
        cameraAngleOffset2D = m3TargetData[3];
      }
    }
    else if(m3TargetData[4] == 1 && m4TargetData[4] == 0){
      hasTarget = true;
      usingM3Camera = true;
      cameraXOffset = m3TargetData[0];
      cameraYOffset = m3TargetData[1];
      cameraAngleOffset3D = m3TargetData[2];
      cameraAngleOffset2D = m3TargetData[3];
    }
    else if (m4TargetData[4] ==1 && m3TargetData[4] == 0){
      //System.out.println(true);
      hasTarget = true;
      usingM3Camera = false;
      cameraXOffset = m4TargetData[0];
      cameraYOffset = m4TargetData[1];
      cameraAngleOffset3D = m4TargetData[2];
      cameraAngleOffset2D = m4TargetData[3];
    }
    else{
      //System.out.println("4st");
      hasTarget = false;
    }
    if (hasTarget && enabled){
      //x and y are working
      if (Math.abs(cameraAngleOffset2D)>60){
        robotAngleOffset = 0;
      }
      m_swerveSubsystem.setDriveCommandDisabled(true);
      m_swerveSubsystem.reefControlledDrive(robotXOffset, robotYOffset, robotAngleOffset, x, y,enabled);
    }else{
      m_swerveSubsystem.setDriveCommandDisabled(false);
      m_swerveSubsystem.reefControlledDrive(0, 0, 0, 0, 0,false);
    }
  }
  // public void intakeAlign(double robotX,double robotY,boolean enabled){
  //   if (intakeTargetData[4] == 1){
  //     if (Math.abs(m3TargetData[3])<Math.abs(m4TargetData[3])){
  //       //System.out.println("1st");
  //       hasTarget = true;
  //       cameraXOffset = m3TargetData[0];
  //       cameraYOffset = m3TargetData[1];
  //       cameraAngleOffset3D = m3TargetData[2];
  //       cameraAngleOffset2D = m3TargetData[3]; 
  //     }
  //   }
  //   else{
  //     //System.out.println("4st");
  //     hasTarget = false;
  //   }
  //   if (hasTarget && enabled){
  //     m_swerveSubsystem.reefControlledDrive(0, cameraXOffset, cameraAngleOffset3D, robotX, robotY, enabled);
  //   }
  //   else{
  //     m_swerveSubsystem.setDriveCommandDisabled(false);

  //     m_swerveSubsystem.reefControlledDrive(0, 0, 0, 0, 0, false);
  //   }
  // }
  public void resetCount(){
    m_swerveSubsystem.resetCount();
  }
  public boolean getHasTarget(){
    return hasTarget;
  }
  public boolean getAtTargetPosition(){
    return m_swerveSubsystem.getAtTargetPosition(hasTarget);
  }
  public boolean hasRightID(int id){
    if (hasID(m3CameraResult,id) || hasID(m3CameraResult,id)){
      return true;
    }
    else{
      return false;
    }
  }
  public boolean hasID(PhotonPipelineResult pipeline, int idd){
    if(pipeline.hasTargets() && pipeline.getTargets().stream().anyMatch(t -> Arrays.asList(idd).contains(t.getFiducialId()))){
      return true;
    }
    else{
      return false;
    }
  }
  public void setDriveCommandDisabled(boolean value){
    m_swerveSubsystem.setDriveCommandDisabled(value);
  }
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator pose,Pose2d prevEstimatedRobotPose,PhotonPipelineResult camResult) {
  //       pose.setReferencePose(prevEstimatedRobotPose);
  //       return pose.update(camResult);
  //   }
  @Override
  public void periodic() {
    //m_swerveSubsystem.getTargetID();

    m3Pose.addHeadingData(m_swerveSubsystem.getAutoTime(), m_swerveSubsystem.getAutoRotation());//buffer saves last second value, getAutoTime to remove latency
    m4Pose.addHeadingData(m_swerveSubsystem.getAutoTime(), m_swerveSubsystem.getAutoRotation());
    m3CameraResult = m3Camera.getLatestResult();
    m4CameraResult = m4Camera.getLatestResult();
    m3TargetData = getData(m3CameraResult);
    m4TargetData = getData(m4CameraResult);
    
    if (m3CameraResult.hasTargets()){
    try{
      m3RobotPose  = m3Pose.update(m3CameraResult).get();
      //System.out.println("M3");
      m_swerveSubsystem.updateVisionPoseEstimator(m3RobotPose.estimatedPose.toPose2d(),m3RobotPose.timestampSeconds);
      SmartDashboard.putNumber("M3Pose2dx", m3RobotPose.estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("M3Pose2dy", m3RobotPose.estimatedPose.toPose2d().getY());
    }catch (Exception e) {
      //System.out.println("PoseEstimator(M3 HAS NO TARGET) "+e);
    }
      
    }
    if (m4CameraResult.hasTargets()){
      try{
        m4RobotPose  =  m4Pose.update(m4CameraResult).get();
        //System.out.println("M4");
        m_swerveSubsystem.updateVisionPoseEstimator(m4RobotPose.estimatedPose.toPose2d(),m4RobotPose.timestampSeconds);
        SmartDashboard.putNumber("M4Pose2dx", m4RobotPose.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("M4Pose2dy", m4RobotPose.estimatedPose.toPose2d().getY());
      }catch(Exception e){
        //System.out.println("PoseEstimator(M4 HAS NO TARGET) "+e);
      }
      
    }
    if (alignActive){
      System.out.println("align active");
      align(xTarget,yTarget , id, true);
    }
    

    //align(0, 1, 20, true);
    //System.out.println(m3Camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX());
    //System.out.println(m3CameraResult.size());
    //System.out.println(m3Camera.getLatestResult().getBestTarget().getBestCameraToTarget().getRotation().getAngle()  * (180/Math.PI));
    //System.out.println(m3Camera.getLatestResult().getBestTarget().getYaw());
    
    //align(0, 0, id, false);
    //align(0,1,20,true);
    //System.out.println(m3Camera.getLatestResult().getTargets().get(0).getYaw());
    
    double z = Math.sqrt(Math.pow(cameraXOffset,2) + Math.pow(cameraYOffset,2));
    //System.out.println(usingM3Camera);
    if (usingM3Camera){
      //System.out.println(true);
      robotAngleOffset = cameraAngleOffset3D + 40;
      //System.out.println(robotAngleOffset);
      Vector vx = new Vector(cameraXOffset,(320  + robotAngleOffset) % 360,true);
      //System.out.println(vx.getX());
      Vector vy = new Vector(0,0,true);
      if (cameraYOffset<0){
        vy = new Vector(Math.abs(cameraYOffset),50 + robotAngleOffset,true);
      }
      else if (cameraYOffset>0){
        vy = new Vector(Math.abs(cameraYOffset),230 + robotAngleOffset,true);
      }
      Vector combinedVector = vx.addVector(vy);
      robotYOffset = combinedVector.getY();
      robotXOffset = combinedVector.getX()-0.28;
    }
    else{
      robotAngleOffset = cameraAngleOffset3D - 44;
      Vector vx = new Vector(cameraXOffset, 40 + robotAngleOffset,true);
      Vector vy = new Vector(0,0,true);
      if (cameraYOffset<0){
        vy = new Vector(Math.abs(cameraYOffset),130 + robotAngleOffset,true);
      }
      else if (cameraYOffset>0){
        vy = new Vector(Math.abs(cameraYOffset),310 + robotAngleOffset,true);
      }
      Vector combinedVector = vx.addVector(vy);
      robotYOffset = combinedVector.getY();
      robotXOffset = combinedVector.getX() + 0.28;
    }
    SmartDashboard.putNumber("xrobot", robotXOffset);
    SmartDashboard.putNumber("yrobot",robotYOffset);
    SmartDashboard.putNumber("3d angle", robotAngleOffset);
    //System.out.println(m4TargetData[1]);
    //System.out.println(m_swerveSubsystem.getAutoRotation().getDegrees());
    
    //System.out.println(m_swerveSubsystem.getAutoTime());
    try{
      //EstimatedRobotPose m3RobotPose  = m3Pose.update(m3CameraResult).get();
      //m_swerveSubsystem.updateVisionPoseEstimator(m3RobotPose.estimatedPose.toPose2d(),m3RobotPose.timestampSeconds);
      
    }catch(Exception e){
      //System.out.println("PoseEstimator(M3 HAS NO TARGET) "+e);
    }
    try {
      //EstimatedRobotPose m4RobotPose = m4Pose.update(m4CameraResult).get();
      //m_swerveSubsystem.updateVisionPoseEstimator(m4RobotPose.estimatedPose.toPose2d(),m4RobotPose.timestampSeconds);
      
    } catch (Exception e) {
      //System.out.println("PoseEstimator(M4 HAS NO TARGET) "+e);
    }
  }
}