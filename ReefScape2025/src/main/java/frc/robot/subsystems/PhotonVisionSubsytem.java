// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.opencv.core.Mat;
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

    leftCamera = new PhotonCamera("x");
    rightCamera = new PhotonCamera("y");
    intakeCamera = new PhotonCamera("z");

  }
  public void alignToTarget(boolean enabled, boolean isAtomonous){
    if (!isAtomonous){
      if (leftCameraHasTarget && rightCameraHasTarget && enabled){
          rightCameraxOffset = rightCameraTargetInfo.getY();
          rightCamerayOffset =  Math.sqrt(Math.pow(rightCameraTargetInfo.getY(), 2) - Math.pow(rightCameraTargetInfo.getX(), 2));
          if(rightCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
            rightCameraAngleOffset = (-1)*(180+rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
          }
          else{
            rightCameraAngleOffset = (180-rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
          }
          leftCameraxOffset = leftCameraTargetInfo.getY();
          leftCamerayOffset =  Math.sqrt(Math.pow(leftCameraTargetInfo.getY(), 2) - Math.pow(leftCameraTargetInfo.getX(), 2));
          if(leftCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
            leftCameraAngleOffset = (-1)*(180+leftCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
          }
          else{
        leftCameraAngleOffset = (180-leftCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
      }
        swerveSubsystem.setDriveCommandDisabled(enabled);
        if (leftCameraAngleOffset>rightCameraAngleOffset){
        
        swerveSubsystem.reefControlledDrive(rightCameraxOffset, rightCamerayOffset,rightCameraAngleOffset,0,0.5, enabled);
        System.out.println("Using Right Cam");
        }else{
          swerveSubsystem.reefControlledDrive(leftCameraxOffset, leftCamerayOffset,leftCameraAngleOffset,0,0.5, enabled);
          System.out.println("Using Left Cam");

        }
      }else if (rightCameraHasTarget && enabled){
        rightCameraxOffset = rightCameraTargetInfo.getY();
        rightCamerayOffset =  Math.sqrt(Math.pow(rightCameraTargetInfo.getY(), 2) - Math.pow(rightCameraTargetInfo.getX(), 2));

        if(rightCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
          rightCameraAngleOffset = (-1)*(180+rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
        }
        else{
          rightCameraAngleOffset = (180-rightCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
        }
        
        //32.253
      swerveSubsystem.setDriveCommandDisabled(enabled);
      swerveSubsystem.reefControlledDrive(rightCameraxOffset, rightCamerayOffset,rightCameraAngleOffset,0,0.5, enabled);
      System.out.println("Using Right Cam");
    }else if (leftCameraHasTarget && enabled){
      leftCameraxOffset = leftCameraTargetInfo.getY();
      leftCamerayOffset =  Math.sqrt(Math.pow(leftCameraTargetInfo.getY(), 2) - Math.pow(leftCameraTargetInfo.getX(), 2));
      if(leftCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
        leftCameraAngleOffset = (-1)*(180+leftCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
      }
      else{
        leftCameraAngleOffset = (180-leftCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
      }
      
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
        intakeCameraxOffset = intakeCameraTargetInfo.getY();
        intakeCamerayOffset = intakeCameraTargetInfo.getX();
        if(intakeCameraTargetInfo.getRotation().getZ() * (180/Math.PI)<0){
          intakeCameraAngleOffset = (-1)*(180+intakeCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
        }
        else{
          intakeCameraAngleOffset = (180-intakeCameraTargetInfo.getRotation().getZ()*(180/Math.PI));
        }
        swerveSubsystem.setDriveCommandDisabled(enabled);
        swerveSubsystem.reefControlledDrive(intakeCameraxOffset, intakeCamerayOffset,intakeCameraAngleOffset,0,0.5, enabled);
        System.out.println("Using Intake Cam");
    }else{
      swerveSubsystem.reefControlledDrive(0, 0, 0, 0, 0, false);
      System.out.println("NO TARGETS");
    }
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
