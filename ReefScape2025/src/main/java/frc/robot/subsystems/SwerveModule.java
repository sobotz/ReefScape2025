package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class SwerveModule {
    //motors
    TalonFX driveMotor;
    TalonFX turnMotor;
    //CAN coder
    CANcoder axisSensor;
    //degree when rotating
    double rotationalDegreeValue;
    //vector used for calculating the exact angle and magnitude
    Vector strafeVector;
    Vector rotationalVector;
    Vector driveVector;
    //Current module values
    double currentModuleDegree;
    double currentRobotDegree;
    //Target module values
    double targetModuleDegree;
    double targetModuleMagnitude;

    double degreeOffset;
    //PID Controller
    PIDController degreeController;
    PIDController velocityController;
    

    boolean inverted;
    public SwerveModule(TalonFX dMotor, TalonFX tMotor, CANcoder aSensor, double rotationalDegreeValue,double degreeOffset){
        //Motors
        this.driveMotor = dMotor;
        this.turnMotor = tMotor;
        //CAN coder from swerve subsystem
        this.axisSensor = aSensor;
        //value when rotating
        this.rotationalDegreeValue = rotationalDegreeValue;
        //driving vectors
        strafeVector = new Vector(0,0);
        rotationalVector = new Vector(0,0);
        //PID 

        degreeController = new PIDController(0.01, 0.00,0.00);/// tuned properly make sure the calculate is about 0
        //degreeController.setTolerance(.05);

        degreeController.enableContinuousInput(0,360);
        //inverted module wheel direction and magnitude
        inverted = false;
        //CANcoder offset to have 0 degrees align with the robot
        this.degreeOffset = degreeOffset;
    }
    public void drive(Vector strafeVector, double rotationalMagnitude, double currentRobotDegree, boolean relativeVelocityControl,boolean disable,boolean climbMode){
        this.currentRobotDegree = currentRobotDegree;
        //setting and creating the strafe vector and rotational vector
        //System.out.println(relativeVelocityControl);
        this.strafeVector = strafeVector;
        this.rotationalVector = new Vector(rotationalMagnitude, rotationalDegreeValue,true);
        
        
        /*if (Math.abs(rotationalVector.getMagnitude())>0.5){
            rotationalVector.setMagnitude(0.5 *(rotationalVector.getMagnitude()/Math.abs(rotationalVector.getMagnitude())));
            if (strafeVector.getMagnitude()>0.5){
                strafeVector.setMagnitude(0.5 * (strafeVector.getMagnitude()/Math.abs(strafeVector.getMagnitude())));
            }
        }
        else{ 
            if ((strafeVector.getMagnitude() + rotationalVector.getMagnitude())>1){
                strafeVector.setMagnitude(1-Math.abs(rotationalVector.getMagnitude()));
            }
            
        }*/

        //combining vectors
        if(relativeVelocityControl == false){
            strafeVector = new Vector(strafeVector.getMagnitude(),strafeVector.getDegrees()-currentRobotDegree,true);
        }
        driveVector = strafeVector.addVector(rotationalVector);
        //current wheel degree  0 offsetted
        currentModuleDegree = (((((axisSensor.getPosition().getValueAsDouble()) % 1) + 1) % 1) * 360);
        currentModuleDegree = (((currentModuleDegree - degreeOffset) + 360) % 360);

        targetModuleMagnitude = driveVector.getMagnitude();
        targetModuleDegree = driveVector.getDegrees();
            if (inverted){
                targetModuleMagnitude *= -1;
                targetModuleDegree = ((targetModuleDegree + 180)%360);
            }
        
        //field orientation
        
        //the target values put into variables from a vector;
    
        //need to call calculate to determine the degree error
        degreeController.calculate(currentModuleDegree,targetModuleDegree);
        //if rotation is greater than 90 then inverted true
        if (Math.abs(degreeController.getError())>90){
            inverted = !inverted;
        }
        //PID controller
        
        if (disable) {
            turnMotor.set(0);
            driveMotor.set(0);
            
        }
        else if (climbMode){
            System.out.println("WORKING");
            turnMotor.set(-degreeController.calculate(currentModuleDegree,0));
            driveMotor.set(0);
        }
        else if((Math.abs(driveVector.getMagnitude()) < 0.001)&&(Math.abs(rotationalMagnitude)<0.001)){
            turnMotor.set(0);
            driveMotor.set(0);
        }
        else{
            turnMotor.set(-degreeController.calculate(currentModuleDegree,targetModuleDegree));
            driveMotor.set(targetModuleMagnitude);
        }

    }
    public double getRawValue(){
        return (((((axisSensor.getPosition().getValueAsDouble()) % 1) + 1) % 1) * 360);
    }
    public double getTarget(){
        return targetModuleDegree;
    }
    public double getDriveSensorPosition(boolean isRedAlliance){
        // if (isRedAlliance){
        //     return -driveMotor.getPosition().getValueAsDouble()/19.607;
        // }
        // else{
        //     return driveMotor.getPosition().getValueAsDouble()/19.607;
        // }
        return driveMotor.getPosition().getValueAsDouble()/19.607;
        //25.97290039;//* Constants.SwerveConstants.wheelRotationPerMotorRotation*(2 * Math.PI* Constants.SwerveConstants.wheelRadius);//CHANGE
    }
    public double getRawDriveSensorPosition(){
        return driveMotor.getPosition().getValueAsDouble();
    }
    public double getDriveSensorVelocity(boolean isRedAlliance){
        // if (isRedAlliance){
        //     System.out.println(-driveMotor.getVelocity().getValueAsDouble()/19.607);
        //     return -driveMotor.getVelocity().getValueAsDouble()/19.607;
        // }
        // else{
        //     return driveMotor.getVelocity().getValueAsDouble()/19.607;
        // }
        return driveMotor.getVelocity().getValueAsDouble()/19.607;
        //25.97290039);// * Constants.SwerveConstants.wheelRotationPerMotorRotation)/60) * 2 * Math.PI * Constants.SwerveConstants.wheelRadius);//CHANGE
    }
    public double getCurrentModuleDegree(){
        return currentModuleDegree;
    }
    public Rotation2d get2dCurrentModuleDegree(boolean velocityCheck){
        Rotation2d x = new Rotation2d(0);
        //if (velocityCheck){
        //    x = new Rotation2d((((currentModuleDegree + currentRobotDegree) % 360) * Math.PI)/180);
        //}
        //else{
        x = new Rotation2d((((currentModuleDegree) % 360) * Math.PI)/180);
        //}
        
        // if (isRedAlliance){
        //     x = new Rotation2d(Math.toRadians((x.getDegrees() + 180) % 360));
        // }
        return x;
    }
    public SwerveModulePosition getSwerveModulePosition(boolean isRedAlliance){
        return new SwerveModulePosition((getDriveSensorPosition(isRedAlliance)),get2dCurrentModuleDegree(false));
    }
    public SwerveModuleState getSwerveModuleState(boolean isRedAlliance){
        return new SwerveModuleState(getDriveSensorVelocity(isRedAlliance), get2dCurrentModuleDegree(true));// speed meters per second , rotation 2d angle
    }
}
