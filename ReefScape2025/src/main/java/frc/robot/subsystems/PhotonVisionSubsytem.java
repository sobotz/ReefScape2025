// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  SwerveSubsystem swerveSubsystem;
  //Cameras
   PhotonCamera leftCamera;
   PhotonCamera rightCamera;
   PhotonCamera intakeCamera;
   //Camera Data
   PhotonPipelineResult leftCameraResult;
   PhotonPipelineResult rightCameraResult;
   PhotonPipelineResult intakeCameraResult;

   Transform3d leftCameraTargetInfo;
   Transform3d rightCameraTargetInfo;
   Transform3d intakeCameraTargetInfo;

   double leftCameraxOffset;
   double leftCamerayOffset;
   double leftCameraAngleOffset;
   double rightCameraxOffset;
   double rightCamerayOffset;
   double rightCameraAngleOffset;
   double intakeCameraxOffset;
   double intakeCamerayOffset;
   double  intakeCameraAngleOffset;

   List<PhotonTrackedTarget> leftCameraTargets;
   List<PhotonTrackedTarget> rightCameraTargets;
   List<PhotonTrackedTarget> intakeCameraTargets;

   PhotonTrackedTarget leftCameraCurrentTarget;
   PhotonTrackedTarget rightCameraCurrentTarget;
   PhotonTrackedTarget intakeCameraCurrentTarget;

   boolean leftCameraHasTarget;
   boolean rightCameraHasTarget;
   boolean intakeCameraHasTarget;

   int leftCameraTargetId;
   int rightCameraTargetId;
   int intakeCameraTargetId;



  public PhotonVisionSubsytem() {
    swerveSubsystem = new SwerveSubsystem();

    leftCamera = new PhotonCamera(Constants.PhotonVisionConstants.leftCameraName);
    rightCamera = new PhotonCamera(Constants.PhotonVisionConstants.rightCameraName);
    intakeCamera = new PhotonCamera(Constants.PhotonVisionConstants.rightCameraName);

  }
  public void alignToTarget(boolean enabled, boolean isAtomonous){
    if (!isAtomonous){
      if (leftCameraHasTarget && rightCameraHasTarget && enabled){
          rightCameraxOffset = getCameraOffsets(rightCameraTargetInfo, false,false)[0];
          rightCamerayOffset =getCameraOffsets(rightCameraTargetInfo, false,false)[1];
          rightCameraAngleOffset = getCameraOffsets(rightCameraTargetInfo, false,false)[2];
          leftCameraxOffset = getCameraOffsets(leftCameraTargetInfo, false,false)[0];
          leftCamerayOffset =getCameraOffsets(leftCameraTargetInfo, false,false)[1];
          leftCameraAngleOffset = getCameraOffsets(leftCameraTargetInfo, false,false)[2];
        swerveSubsystem.setDriveCommandDisabled(enabled);
        if (Math.abs(leftCameraAngleOffset)>Math.abs(rightCameraAngleOffset)){
        swerveSubsystem.reefControlledDrive(rightCameraxOffset, rightCamerayOffset,rightCameraAngleOffset,0,0.5, enabled);
        System.out.println("Using Right Cam");
        }else{
          swerveSubsystem.reefControlledDrive(leftCameraxOffset, leftCamerayOffset,leftCameraAngleOffset,0,0.5, enabled);
          System.out.println("Using Left Cam");

        }
      }else if (rightCameraHasTarget && enabled){
        rightCameraxOffset = getCameraOffsets(rightCameraTargetInfo, false,false)[0];
        rightCamerayOffset =getCameraOffsets(rightCameraTargetInfo, false,false)[1];
        rightCameraAngleOffset = getCameraOffsets(rightCameraTargetInfo, false,false)[2];
        
        //32.253
      swerveSubsystem.setDriveCommandDisabled(enabled);
      swerveSubsystem.reefControlledDrive(rightCameraxOffset, rightCamerayOffset,rightCameraAngleOffset,0,0.5, enabled);
      System.out.println("Using Right Cam");
    }else if (leftCameraHasTarget && enabled){
      leftCameraxOffset = getCameraOffsets(leftCameraTargetInfo, false,true)[0];
      leftCamerayOffset =getCameraOffsets(leftCameraTargetInfo, false,true)[1];
      leftCameraAngleOffset = getCameraOffsets(leftCameraTargetInfo, false,true)[2];
      
    swerveSubsystem.setDriveCommandDisabled(enabled);
    swerveSubsystem.reefControlledDrive(leftCameraxOffset, leftCamerayOffset,leftCameraAngleOffset,0,0.5, enabled);
    System.out.println("Using Left Cam");
  }
      else{
        swerveSubsystem.reefControlledDrive(0, 0, 0, 0, 0, false);
        System.out.println("NO TARGETS");
      }
    }else{
      if (intakeCameraHasTarget && enabled){
        intakeCameraxOffset = getCameraOffsets(intakeCameraTargetInfo,true,false)[0];
        intakeCamerayOffset = getCameraOffsets(intakeCameraTargetInfo,true,false)[1];
        intakeCameraAngleOffset = getCameraOffsets(intakeCameraTargetInfo,true,false)[2];
        swerveSubsystem.setDriveCommandDisabled(enabled);
        swerveSubsystem.reefControlledDrive(intakeCameraxOffset, intakeCamerayOffset,intakeCameraAngleOffset,0,0.5, enabled);
        System.out.println("Using Intake Cam");
    }else{
      swerveSubsystem.reefControlledDrive(0, 0, 0, 0, 0, false);
      System.out.println("NO TARGETS");
    }
    }
  }
  public double[] getCameraOffsets(Transform3d targetInfo,boolean cameraIsParallel,boolean isLeftCamera){
      if (isLeftCamera){
      double zAngleOffset = 0.0;
      double angleoffset = 0;
       angleoffset =targetInfo.getRotation().getAngle();
      if (targetInfo.getRotation().getZ() * (180/Math.PI)<0){
        zAngleOffset = (-1)*(180+targetInfo.getRotation().getZ()*(180/Math.PI));
      }else{
        zAngleOffset = (180-targetInfo.getRotation().getZ()*(180/Math.PI));
      }
      double x = targetInfo.getX()*Math.sin(angleoffset+Constants.PhotonVisionConstants.leftCameraCenterOffset);//d1
      double y = 0.0;
      if (cameraIsParallel){
      y = targetInfo.getX();
      }else{
        y = targetInfo.getX()*Math.cos(angleoffset+Constants.PhotonVisionConstants.leftCameraCenterOffset);//d2
      }
      double[] info = {x,y,zAngleOffset};
      return info;
    }else{
      double zAngleOffset = 0.0;
      double angleoffset = 0;
       angleoffset =targetInfo.getRotation().getAngle();
      if (targetInfo.getRotation().getZ() * (180/Math.PI)<0){
        zAngleOffset = (-1)*(180+targetInfo.getRotation().getZ()*(180/Math.PI));
      }else{
        zAngleOffset = (180-targetInfo.getRotation().getZ()*(180/Math.PI));
      }
      double x = targetInfo.getX()*Math.sin(angleoffset+Constants.PhotonVisionConstants.rightCameraCenterOffset);//d1
      double y = 0.0;
      if (cameraIsParallel){
      y = targetInfo.getX();
      }else{
        y = targetInfo.getX()*Math.cos(angleoffset+Constants.PhotonVisionConstants.rightCameraCenterOffset);//d2
      }
      double[] info = {x,y,zAngleOffset};
      return info;
    }
  }
  @Override
  public void periodic() {
    leftCameraResult = leftCamera.getLatestResult();
    rightCameraResult = rightCamera.getLatestResult();
    intakeCameraResult = intakeCamera.getLatestResult();

    leftCameraTargets = leftCameraResult.getTargets();
    rightCameraTargets = rightCameraResult.getTargets();
    intakeCameraTargets = intakeCameraResult.getTargets();

    leftCameraHasTarget = leftCameraResult.hasTargets();
    rightCameraHasTarget = rightCameraResult.hasTargets();
    intakeCameraHasTarget = intakeCameraResult.hasTargets();
    
      try {
        leftCameraCurrentTarget = leftCameraTargets.get(0);
        leftCameraTargetInfo = leftCameraCurrentTarget.getBestCameraToTarget();
        leftCameraTargetId = leftCameraCurrentTarget.getFiducialId();
        SmartDashboard.putNumber("LEFT CAM XOFFSET", leftCameraxOffset);
        SmartDashboard.putNumber("LEFT CAM YOFFSET", leftCamerayOffset);
        SmartDashboard.putNumber("LEFT CAM ANGLE OFFSET",leftCameraAngleOffset);
      } catch (Exception e) {
        System.out.println(e);
      }
      
    
    if(rightCameraHasTarget){
      rightCameraCurrentTarget = rightCameraTargets.get(0);
      rightCameraTargetInfo = rightCameraCurrentTarget.getBestCameraToTarget();
      rightCameraTargetId = rightCameraCurrentTarget.getFiducialId();
      SmartDashboard.putNumber("RIGHT CAM XOFFSET", rightCameraxOffset);
      SmartDashboard.putNumber("RIGHT CAM YOFFSET", rightCamerayOffset);
      SmartDashboard.putNumber("RIGHT CAM ANGLE OFFSET",rightCameraAngleOffset);
    }
    if (intakeCameraHasTarget){
      intakeCameraCurrentTarget = intakeCameraTargets.get(0);
      intakeCameraTargetInfo = intakeCameraCurrentTarget.getBestCameraToTarget();
      intakeCameraTargetId = intakeCameraCurrentTarget.getFiducialId();
      SmartDashboard.putNumber("INTAKE CAM XOFFSET", intakeCameraxOffset);
      SmartDashboard.putNumber("INTAKE CAM YOFFSET", intakeCamerayOffset);
      SmartDashboard.putNumber("INTAKE CAM ANGLE OFFSET",intakeCameraAngleOffset);
    }
    //rightCameraTargetId = leftCameraCurrentTarget.getFiducialId();
    //leftCameraCurrentTarget = leftCameraTargets.get(0);
    
    
    SmartDashboard.putBoolean("LEFT CAM HAS TARGET", leftCameraHasTarget);
    SmartDashboard.putBoolean("RIGHT CAM HAS TARGET",rightCameraHasTarget);
    SmartDashboard.putBoolean("INTAKE CAM HAS TARGET",intakeCameraHasTarget);

    
    
    
  }

}