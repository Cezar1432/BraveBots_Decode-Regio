package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math;


import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers.PinpointV1;

public class Pose {

   public double x;
   public double y;
   public double theta;
   public Pose(double x, double y, double theta){
       this.x= x;
       this.y= y;
       this.theta= PinpointV1.normalizeHeading(theta);

   }

   public Pose(double x, double y){
       this.x= x;
       this.y= y;
       this.theta= 0;
   }
   public Pose(){
       this.x= 0;
       this.y= 0;
       this.theta= 0;
   }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y){
       this.y=y;
    }
    public void setTheta(double theta){
       this.theta= theta;
    }
    public double getX(){
       return this.x;
    }
    public double getY(){
       return this.y;
    }
    double normalize(double h){
       while (h> Math.PI) h-=2* Math.PI;
       while (h< -Math.PI) h+=2* Math.PI;
       return h;
    }
    public double getTheta(){
       return this.theta;
    }
    public double getNormalizedTheta(){
       return normalize(this.theta);
    }
    public Pose getPose(){
       return this;
    }
    public static double[] polarToCartesian(double r, double theta) {
        return new double[]{r * Math.cos(theta), r * Math.sin(theta)};
    }
    public static double[] cartesianToPolar(double x, double y) {
        return new double[] {Math.sqrt(x * x + y * y), MathStuff.normalizeRadians(Math.atan2(y, x))};
    }
}

