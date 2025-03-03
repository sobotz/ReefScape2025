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
public class PhotonVisionSubsytem extends SubsystemBase {
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



  public PhotonVisionSubsytem(SwerveSubsystem subsystem) {
    m_swerveSubsystem = subsystem;
    m3Camera = new PhotonCamera(Constants.PhotonVisionConstants.m3CameraName);
    m4Camera = new PhotonCamera(Constants.PhotonVisionConstants.m4CameraName);
    intakeCamera = new PhotonCamera(Constants.PhotonVisionConstants.m4CameraName);
    id = 1;
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
  public double[] getData(PhotonPipelineResult result){
    if(m3CameraResult.hasTargets() && m3CameraResult.getTargets().stream().anyMatch(t -> Arrays.asList(id).contains(t.getFiducialId()))){
      // get all targets
      m3CameraResult.getTargets().stream()
        // filter out all except id
        .filter(t -> Arrays.asList(id).contains(t.getFiducialId()))
        // set offsets of id
        .forEach(target -> {
          tempXOffset = target.getBestCameraToTarget().getX();//distance to target front back
          tempYOffset = target.getBestCameraToTarget().getY();//distance to target left right
          tempAngleOffset3D = target.getBestCameraToTarget().getRotation().getZ();
          tempAngleOffset2D = target.getBestCameraToTarget().getRotation().getAngle() * (180/Math.PI);//CHECK IF GET ANGLE WORKS AS INTENDED
          tempHasTarget = 1;});
    }
    else{
      tempXOffset = 0;
      tempYOffset = 0;
      tempAngleOffset3D = 0;
      tempAngleOffset2D = 0;
      tempHasTarget = 0;
    }
    double[] returnList ={tempXOffset,tempYOffset,cameraAngleOffset3D,cameraAngleOffset2D, tempHasTarget};
    return returnList; 
  }
  public void align(double x, double y,boolean enanbled){
    
    if (Math.abs(m3TargetData[3])<Math.abs(m4TargetData[3])){
      hasTarget = true;
      usingM3Camera = true;
      cameraXOffset = m3TargetData[0];
      cameraYOffset = m3TargetData[1];
      cameraAngleOffset3D = m3TargetData[2];
      cameraAngleOffset2D = m3TargetData[3];
    }
    else if (Math.abs(m3TargetData[3])>Math.abs(m4TargetData[3])){
      hasTarget = true;
      usingM3Camera = false;
      cameraXOffset = m4TargetData[0];
      cameraYOffset = m4TargetData[1];
      cameraAngleOffset3D = m4TargetData[2];
      cameraAngleOffset2D = m4TargetData[3];
    }
    else if ((Math.abs(m3TargetData[3]) == Math.abs(m4TargetData[3])) && m3TargetData[3] != 0){
      hasTarget = true;
      usingM3Camera = true;
      cameraXOffset = m3TargetData[0];
      cameraYOffset = m3TargetData[1];
      cameraAngleOffset3D = m3TargetData[2];
      cameraAngleOffset2D = m3TargetData[3];
    }
    else{
      hasTarget = false;
    }
    double z = Math.sqrt(Math.pow(cameraXOffset,2) + Math.pow(cameraYOffset,2));
    if (usingM3Camera){
      robotYOffset = z * Math.cos(cameraAngleOffset2D - 50);
      robotXOffset = z * Math.sin(cameraAngleOffset2D - 50) + 0.28;//0.3 meters offset from the center//CHANGE TO REAL METER OFFSET
      robotAngleOffset = cameraAngleOffset3D - 50;
    }
    else{
      robotYOffset = z * Math.cos(cameraAngleOffset2D + 50);
      robotXOffset = z * Math.sin(cameraAngleOffset2D + 50) - 0.28;
      robotAngleOffset = cameraAngleOffset3D + 50;
    }
    if (hasTarget && enanbled){
      m_swerveSubsystem.reefControlledDrive(robotXOffset, robotYOffset, robotAngleOffset, xTarget, yTarget,true);
    }
    m_swerveSubsystem.reefControlledDrive(0, 0, 0, 0, yTarget,false);
    
  }
  @Override
  public void periodic() {
    id = m_swerveSubsystem.getTargetID();
    m3CameraResult = m3Camera.getLatestResult();
    m4CameraResult = m4Camera.getLatestResult();
    intakeCameraResult = intakeCamera.getLatestResult();
    m3TargetData = getData(m3CameraResult);
    m4TargetData = getData(m4CameraResult);
    intakeTargetData = getData(intakeCameraResult);
  }

}