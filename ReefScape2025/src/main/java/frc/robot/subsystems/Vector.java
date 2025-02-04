package frc.robot.subsystems;

public class Vector {
    double magnitude;
    double degrees;
    double x;
    double y;
    //magnitude vector creation
    //takes in any magnitude (negative or position) and any degree (could be beyond 360 degrees)
    public Vector(double magnitude, double degrees, boolean magnitudeVectorCreation){
        this.magnitude = magnitude;
        this.degrees = degrees;
        if (this.magnitude < 0){
            this.degrees = (this.degrees + 180) % 360;
            this.magnitude *= -1;
        }
        if (this.degrees < 0){
            this.degrees = (this.degrees + 3600) % 360;
        }
        //Convert into unit circle (0 is horizontal for unit circles while 0 is vertical on swerve drive)
        //degrees = (this.degrees + 90) % 360;
        //trig 
        x = this.magnitude * Math.cos(Math.toRadians((this.degrees + 90) % 360));
        y = this.magnitude * Math.sin(Math.toRadians((this.degrees + 90) % 360));
    }
    //x and y vector creation
    public Vector(double x, double y){
        //Pythagorean theorem
        this.x = x;
        this.y = y;
        if (x == 0 && y == 0){
            magnitude = 0;
            degrees = 0;
        }
        else{
            this.magnitude = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            //arc tan to convert x and y's to polar degrees
            this.degrees = (Math.toDegrees(Math.atan2(y,x)) + 270)% 360;
        }
        
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getMagnitude(){
        return magnitude;
    }
    public double getDegrees(){
        return degrees;
    }
    public void setMagnitude(double magnitude){
        this.magnitude = magnitude;
        magUpdate(magnitude,degrees);
    }
    public void setDegrees(double degrees){
        this.degrees = degrees;
        magUpdate(magnitude,degrees);
    }
    //Used to update the magnitude and degrees after changing x or y values
    public void xyUpdate(double x, double y){
        this.x = x;
        this.y = y;
        if (x == 0 && y == 0){
            this.magnitude = 0;
            this.degrees = 0;
        }
        else{
            this.magnitude = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            this.degrees = (Math.toDegrees(Math.atan2(y,x)) + 270) % 360;
        }
        
    }
    public void magUpdate(double magnitude, double degrees){
        this.magnitude = magnitude;
        this.degrees = degrees;
        if (this.magnitude < 0){
            this.degrees = (this.degrees + 180) % 360;
            this.magnitude *= -1;
        }
        if (this.degrees < 0){
            this.degrees = (this.degrees + 3600) % 360;
        }
        //Convert into unit circle (0 is horizontal for unit circles while 0 is vertical on swerve drive)
        //this.degrees = (this.degrees + 90) % 360;
        //trig 
        x = this.magnitude * Math.cos(Math.toRadians((this.degrees + 90) % 360));
        y = this.magnitude * Math.sin(Math.toRadians((this.degrees + 90) % 360));
    }
    //Combines vectors together
    public Vector addVector(Vector v){
        double combinedX = x + v.getX();
        double combinedY = y + v.getY();
        if (-0.0000000001<combinedX && combinedX<0.0000000001){
            combinedX = 0;
        }
        if (-0.0000000001<combinedY && combinedY <0.0000000001){
            combinedY = 0;
        }
        return new Vector(combinedX,combinedY);
    }
}
