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

public class PhotonVisionSubsytem extends SubsystemBase {
  //SwereSubsystem
  SwerveSubsystem swerveSubsystem;
  //Cameras
   PhotonCamera leftCamera;
   PhotonCamera rightCamera;
   //Camera Data
   PhotonPipelineResult leftCameraResult;
   PhotonPipelineResult rightCameraResult;

   Transform3d leftCameraTargetInfo;
   Transform3d rightCameraTargetInfo;

   double leftCameraxOffset;
   double leftCamerayOffset;
   double leftCameraAngleOffset;
   double rightCameraxOffset;
   double rightCamerayOffset;
   double rightCameraAngleOffset;

   List<PhotonTrackedTarget> leftCameraTargets;
   List<PhotonTrackedTarget> rightCameraTargets;

   PhotonTrackedTarget leftCameraCurrentTarget;
   PhotonTrackedTarget rightCameraCurrentTarget;

   boolean leftCameraHasTarget;
   boolean rightCameraHasTarget;

   int leftCameraTargetId;
   int rightCameraTargetId;



  public PhotonVisionSubsytem() {
    swerveSubsystem = new SwerveSubsystem();

    leftCamera = new PhotonCamera("Module_3_Arducam_OV2311");
    rightCamera = new PhotonCamera("Module_3_Arducam_OV2311 (1)");

  }
  public void alignToTarget(boolean enabled){
    if (leftCameraHasTarget && rightCameraHasTarget && enabled){
        rightCameraxOffset = rightCameraTargetInfo.getY();
        rightCamerayOffset = rightCameraTargetInfo.getX();
        if(rightCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
          rightCameraAngleOffset = (-1)*(180+rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
        }
        else{
          rightCameraAngleOffset = (180-rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
        }
        
      swerveSubsystem.setDriveCommandDisabled(enabled);
      swerveSubsystem.reefControlledDrive(rightCameraxOffset, rightCamerayOffset,rightCameraAngleOffset-31.25,0,1, enabled);
      System.out.println("Using Right Cam");
    }else if (rightCameraHasTarget && enabled){
      rightCameraxOffset = rightCameraTargetInfo.getY();
      rightCamerayOffset = rightCameraTargetInfo.getX();

      if(rightCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
        rightCameraAngleOffset = (-1)*(180+rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
      }
      else{
        rightCameraAngleOffset = (180-rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
      }
      
      //32.253
    swerveSubsystem.setDriveCommandDisabled(enabled);
    swerveSubsystem.reefControlledDrive(rightCameraxOffset, rightCamerayOffset,rightCameraAngleOffset-31.25,0,1, enabled);
    System.out.println("Using Right Cam");
  }else if (leftCameraHasTarget && enabled){
    leftCameraxOffset = leftCameraTargetInfo.getY();
    leftCamerayOffset = leftCameraTargetInfo.getX();
    if(leftCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
      leftCameraAngleOffset = (-1)*(180+leftCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
    }
    else{
      leftCameraAngleOffset = (180-leftCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
    }
    
  swerveSubsystem.setDriveCommandDisabled(enabled);
  swerveSubsystem.reefControlledDrive(leftCameraxOffset, leftCamerayOffset,leftCameraAngleOffset+31.25,0,1, enabled);
  System.out.println("Using Left Cam");
}
    else{
      swerveSubsystem.reefControlledDrive(0, 0, 0, 0, 0, false);
      System.out.println("NO TARGETS");
    }
  }
  @Override
  public void periodic() {
    leftCameraResult = leftCamera.getLatestResult();
    rightCameraResult = rightCamera.getLatestResult();

    leftCameraTargets = leftCameraResult.getTargets();
    rightCameraTargets = rightCameraResult.getTargets();
  
    leftCameraHasTarget = leftCameraResult.hasTargets();
    rightCameraHasTarget = rightCameraResult.hasTargets();
    
    if (leftCameraHasTarget){
      leftCameraCurrentTarget = leftCameraTargets.get(0);
      leftCameraTargetInfo = leftCameraCurrentTarget.getBestCameraToTarget();
      leftCameraTargetId = leftCameraCurrentTarget.getFiducialId();
      SmartDashboard.putNumber("LEFT CAM XOFFSET", leftCameraxOffset);
      SmartDashboard.putNumber("LEFT CAM YOFFSET", leftCamerayOffset);
      SmartDashboard.putNumber("LEFT CAM ANGLE OFFSET",leftCameraAngleOffset);
    }
    if(rightCameraHasTarget){
      rightCameraCurrentTarget = rightCameraTargets.get(0);
      rightCameraTargetInfo = rightCameraCurrentTarget.getBestCameraToTarget();
      rightCameraTargetId = rightCameraCurrentTarget.getFiducialId();
      SmartDashboard.putNumber("RIGHT CAM XOFFSET", rightCameraxOffset);
      SmartDashboard.putNumber("RIGHT CAM YOFFSET", rightCamerayOffset);
      SmartDashboard.putNumber("RIGHT CAM ANGLE OFFSET",rightCameraAngleOffset);
    }
    
    //rightCameraTargetId = leftCameraCurrentTarget.getFiducialId();
    //leftCameraCurrentTarget = leftCameraTargets.get(0);
    
    
    SmartDashboard.putBoolean("LEFT CAM HAS TARGET", leftCameraHasTarget);
    SmartDashboard.putBoolean("RIGHT CAM HAS TARGET",rightCameraHasTarget);

    
    
    
  }

}
