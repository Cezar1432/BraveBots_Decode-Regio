package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers;


import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.filterParameter;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.xDeceleration;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.yDeceleration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;


public class PinpointV1 implements Localizer {



    public GoBildaPinpointDriver odo;
    public PinpointV1(Robot robot){
        this.odo= robot.odo;
    }
    private double xStrafe, yForward;

    public void resetLocalizer(){
        odo.resetPosAndIMU();
    }

    @Override
    public Pose getPredictedPose() {
        return new Pose(getPredictedX(), getPredictedY());
    }

    public void setOffsets(double xStrafe, double yForward, DistanceUnit unit){
        this.xStrafe= xStrafe;
        this.yForward= yForward;
        odo.setOffsets(this.xStrafe, this.yForward, unit);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();


    }
    public void setOdo(double xStrafe, double yForward, DistanceUnit unit, GoBildaPinpointDriver.EncoderDirection xPodDirection, GoBildaPinpointDriver.EncoderDirection yPodDirection){
        setOffsets(xStrafe, yForward, unit);
        setEncoderDirections(xPodDirection, yPodDirection);
    }

    LowPassFilter xVelocityFilter= new LowPassFilter(filterParameter, 0);
    LowPassFilter yVelocityFilter= new LowPassFilter(filterParameter, 0);

    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xPodDirection, GoBildaPinpointDriver.EncoderDirection yPodDirection){
        odo.setEncoderDirections(xPodDirection, yPodDirection);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    @Override
    public double getXVelocity() {
        return odo.getVelX(DistanceUnit.INCH);
    }

    @Override
    public double getYVelocity() {
        return odo.getVelY(DistanceUnit.INCH);
    }

    @Override
    public double getHeadingVelocity() {
        return odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    @Override
    public double getXYVelocity() {
        return Math.hypot(getXVelocity(), getYVelocity());
    }

    Pose startPose= new Pose();
    Pose pose= new Pose();
    public void setStartingPose(Pose startPose){

        //this.startPose= startPose;
        odo.setPosition(new Pose2D(DistanceUnit.INCH, startPose.x, startPose.y, AngleUnit.RADIANS, startPose.getTheta()));


    }
    Pose currentPosition= new Pose();



    public Pose getCurrentPosition(){
      //  return new Pose(odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS));
        Pose somePose= getActualPose();
        return new Pose(startPose.getX()+ somePose.getX(),  startPose.getY()+ somePose.getY(), normalizeHeading( somePose.getTheta()));

    }


    public Pose getRawPosition(){
        return new Pose(odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS));
    }

    public static double normalizeHeading(double h){

        while(h> Math.PI) h-= 2* Math.PI;
        while(h< -Math.PI) h+= 2* Math.PI;
        return h;
    }


    public static double xRobotVelocity, yRobotVelocity;
    public static double forwardGlide, lateralGlide;
    public static double xGlide, yGlide;


    double heading, x, y, xVelocity, yVelocity;
    public void updateGlide(){

        xRobotVelocity = xVelocity * Math.cos(-heading) - yVelocity * Math.sin(-heading);
        yRobotVelocity = xVelocity * Math.sin(-heading) + yVelocity * Math.cos(-heading);

        forwardGlide = Math.signum(xRobotVelocity) * xRobotVelocity * xRobotVelocity / (2.0 * xDeceleration);
        lateralGlide = Math.signum(yRobotVelocity) * yRobotVelocity * yRobotVelocity / (2.0 * yDeceleration);

        xGlide = forwardGlide * Math.cos(heading) - lateralGlide * Math.sin(heading);
        yGlide = forwardGlide * Math.sin(heading) + lateralGlide * Math.cos(heading);
    }
    public double predictedX, predictedY;
    public double getPredictedX(){
        return this.predictedX;
    }
    public double getPredictedY(){
        return this.predictedY;
    }
    public Pose getActualPose(){
//        double odoX= odo.getPosX(DistanceUnit.INCH);
//        double odoY= odo.getPosY(DistanceUnit.INCH);
//        double odoH= odo.getHeading(AngleUnit.RADIANS);
//
//        double y= odoY* Math.cos(startPose.getTheta()) - odoX* Math.sin(startPose.getTheta());
//        double x= odoX* Math.cos(startPose.getTheta()) + odoY* Math.sin(startPose.getTheta());
//        double heading= startPose.getTheta()+ odoH;
//        heading= normalizeHeading(heading);
       // setTargetVector(y*Math.cos(-heading) - x*Math.sin(-heading) , y*Math.sin(-heading)+x*Math.cos(-heading) , rotation);



        Pose2D p = odo.getPosition();
        double x= p.getX(DistanceUnit.INCH);
        double y= p.getY(DistanceUnit.INCH);
        double heading= p.getHeading(AngleUnit.RADIANS);


        return new Pose(x, y, heading);
    }
    @Override
    public void update(){
        odo.update();
        //currentPosition= new Pose(getActualPose().getX()+ getActualPose().getX(), getActualPose().getY()+ startPose.getY(), getActualPose().getTheta());
        Pose currentPose= getActualPose();
         x= currentPose.getX();
         y= currentPose.getY();
         heading= currentPose.getTheta();
        xVelocity = xVelocityFilter.getValue(odo.getVelX(DistanceUnit.MM));
        yVelocity = yVelocityFilter.getValue(odo.getVelY(DistanceUnit.MM));
        updateGlide();
        predictedX = x - xGlide;
        predictedY = y - yGlide;
    }



}
