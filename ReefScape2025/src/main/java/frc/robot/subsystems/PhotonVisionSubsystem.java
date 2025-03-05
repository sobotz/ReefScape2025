// Copym4 (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class PhotonVisionSubsystem extends SubsystemBase {
  //SwereSubsystem
  SwerveSubsystem m_swerveSubsystem;
  //Cameras
   PhotonCamera m3Camera;
   PhotonCamera m4Camera;
   PhotonCamera intakeCamera;
   //Camera Data
   PhotonPipelineResult m3CameraResult;
   PhotonPipelineResult m4CameraResult;
   PhotonPipelineResult intakeCameraResult;

   Transform3d m3CameraTargetInfo;
   Transform3d m4CameraTargetInfo;
   Transform3d intakeCameraTargetInfo;

   double cameraXOffset;
   double cameraYOffset;
   double cameraAngleOffset3D;
   double cameraAngleOffset2D;

   double robotXOffset;
   double robotYOffset;
   double robotAngleOffset;
  
   double[] m3TargetData;
   double[] m4TargetData;
   double[] intakeTargetData;
  
   double intakeCameraxOffset;
   double intakeCamerayOffset;
   double intakeCameraAngleOffset3D;
   double intakeCameraAngleOffset2D;

   List<PhotonTrackedTarget> m3CameraTargets;
   List<PhotonTrackedTarget> m4CameraTargets;
   List<PhotonTrackedTarget> intakeCameraTargets;

   PhotonTrackedTarget m3CameraCurrentTarget;
   PhotonTrackedTarget m4CameraCurrentTarget;
   PhotonTrackedTarget intakeCameraCurrentTarget;

   double tempXOffset;
   double tempYOffset;
   double tempAngleOffset3D;
   double tempAngleOffset2D;
   double tempHasTarget;

   boolean hasTarget;


   int m3CameraTargetId;
   int m4CameraTargetId;
   int intakeCameraTargetId;
   int id;
   boolean usingM3Camera;

   double xTarget;
   double yTarget;

   boolean atTargetPosition;
   boolean alignActive;


  public PhotonVisionSubsystem(SwerveSubsystem subsystem) {
    atTargetPosition = false;
    m_swerveSubsystem = subsystem;
    xTarget = 0;
    yTarget = 0;
    alignActive = false;
    m3Camera = new PhotonCamera(Constants.PhotonVisionConstants.m3CameraName);
    m4Camera = new PhotonCamera(Constants.PhotonVisionConstants.m4CameraName);
    intakeCamera = new PhotonCamera(Constants.PhotonVisionConstants.m4CameraName);
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
    yTarget = 0;
    

  }
  public void setTargetPosition(double x,double y){
    xTarget = x;
    yTarget = y;
  }
  public void enableAlign(boolean enableAlign,double x, double y, int id){
    xTarget = x;
    yTarget = y;
    alignActive = enableAlign;
    m_swerveSubsystem.setTargetID(id);
    this.id = id;
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

      // robotYOffset = z * Math.cos((Math.PI/180) * (cameraAngleOffset2D - 50));
      // robotXOffset = z * Math.sin((Math.PI/180) * (cameraAngleOffset2D - 50));//0.3 meters offset from the center//CHANGE TO REAL METER OFFSET
      
      //System.out.println("robotX: " + robotXOffset);
    }
    else{
      robotAngleOffset = cameraAngleOffset3D - 42;
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

      // robotYOffset = z * Math.cos((Math.PI/180) * (180-(cameraAngleOffset2D + 50)));
      // robotXOffset = z * Math.sin((Math.PI/180) * (180-(cameraAngleOffset2D + 50))) - 0.28;
      
    }
    //System.out.println(hasTarget);
    if (hasTarget && enabled){
      //x and y are working
      if (Math.abs(cameraAngleOffset2D)>60){
        robotAngleOffset = 0;
      }
      m_swerveSubsystem.setDriveCommandDisabled(enabled);
      m_swerveSubsystem.reefControlledDrive(robotXOffset, robotYOffset, robotAngleOffset, x, y,enabled);
    }else{
      m_swerveSubsystem.setDriveCommandDisabled(false);
      m_swerveSubsystem.reefControlledDrive(robotXOffset, robotYOffset, robotAngleOffset, x, y,false);
    }
   
  }
  public void intakeAlign(double robotX,double robotY,boolean enabled){
      
  }
  public boolean getAtTargetPosition(){
    return m_swerveSubsystem.getAtTargetPosition();
  }
  @Override
  public void periodic() {
    //m_swerveSubsystem.getTargetID();
    if (alignActive){
      //System.out.println("align active");
      align(xTarget,yTarget , id, true);
    }
    m3CameraResult = m3Camera.getLatestResult();
    m4CameraResult = m4Camera.getLatestResult();
    //align(0, 1, 20, true);
    //System.out.println(m3Camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX());
    //System.out.println(m3CameraResult.size());
    
    //System.out.println(m3Camera.getLatestResult().getBestTarget().getBestCameraToTarget().getRotation().getAngle()  * (180/Math.PI));
    //System.out.println(m3Camera.getLatestResult().getBestTarget().getYaw());
    intakeCameraResult = intakeCamera.getLatestResult();
    m3TargetData = getData(m3CameraResult);
    m4TargetData = getData(m4CameraResult);
    intakeTargetData = getData(intakeCameraResult);
    //align(0,1,20,true);
    //System.out.println(m3Camera.getLatestResult().getTargets().get(0).getYaw());
    SmartDashboard.putNumber("xrobot", robotXOffset);
    SmartDashboard.putNumber("yrobot",robotYOffset);
    SmartDashboard.putNumber("3d angle", robotAngleOffset);
    //System.out.println(m4TargetData[1]);
  }
}