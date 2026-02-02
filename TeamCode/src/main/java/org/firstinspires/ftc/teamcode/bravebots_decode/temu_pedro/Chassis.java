package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro;




import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.XYVelocityConstraint;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.dForward;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.dHeading;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.dStrafe;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.distanceConstraint;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.forward;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.forwardThreshold;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.heading;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.headingConstraint;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.headingThreshold;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.headingVelocityConstraint;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.pForward;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.pHeading;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.pStrafe;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryDForward;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryDHeading;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryDStrafe;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryForward;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryHeading;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryPForward;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryPHeading;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryPStrafe;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.secondaryStrafe;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.strafe;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.strafeThreshold;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.threadUpdatePeriod;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.useSecondaryForward;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.useSecondaryHeading;
import static org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants.useSecondaryStrafe;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.MathStuff;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers.Localizer;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers.PinpointV1;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;

public class Chassis implements Runnable{

    BetterMotor frontLeft, frontRight, backLeft, backRight;
    public Localizer localizer;
    public DrivetrainInterface drivetrain;
   // public SwerveDrivetrain swerveDrivetrain;

    private volatile boolean running= false;

    private Thread t;



    @Override
    public void run() {
        while (running && !Thread.currentThread().isInterrupted()) {
            update();
            try {
                Thread.sleep(threadUpdatePeriod);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void startChassisThread(){
        if(!running || !t.isAlive()){
            running = true;
            t = new Thread(this, "MYFollower-Thread");
            t.setDaemon(true);
            t.start();

        }
    }

    public void startChassisThread2(){
        running= true;
        new Thread(()-> {
            while (running && !Thread.currentThread().isInterrupted()) {

                try {
                    update();
                    Thread.sleep(threadUpdatePeriod);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }).start();
    }

    public enum Localizers{
        PINPOINT_V1, PINPOINT_V2
    }

    public void initializeMotors(Robot robot){
        frontLeft= robot.leftFront;
        frontRight= robot.rightFront;
        backLeft= robot.leftBack;
        backRight= robot.rightBack;
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public enum Drivetrain{
        SWERVE, MECANUM
    }
    public Chassis(Robot robot, Control control , Gamepad gp, Localizers localizer, Drivetrain chassis){
        initializeMotors(robot);
        this.gp = gp;
        //localizer= new PinpointV1(robot);
        this.control= control;
        if(localizer== Localizers.PINPOINT_V1)
            this.localizer= new PinpointV1(robot);
        else
            this.localizer= new PinpointV1(robot);

        if(chassis== Drivetrain.MECANUM)
            drivetrain= new MecanumDrivetrain(robot);
        else
            drivetrain= new SwerveDrivetrain(robot);
    }



    volatile Gamepad gp;
    Object p;
    public Chassis(Robot robot, Control control, Gamepad gp, Localizers localizer){
        initializeMotors(robot);
        this.gp = gp;
        //localizer= new PinpointV1(robot);

        this.control= control;
        if (localizer == Localizers.PINPOINT_V1) {
            this.localizer= new PinpointV1(robot);
        }
        else
            this.localizer= new PinpointV1(robot);
    }

    double maxPower= 1;
    public void setMaxPower(double maxPower){
        this.maxPower= maxPower;
    }

    public enum Control{
        AUTO, GAMEPAD;
    }


    public double getDistanceFromTarget(){
        synchronized (this) {
            if (targetPosition != null) {
                return Math.sqrt(Math.pow(targetPosition.getX() - currentPose.getX(), 2) + Math.pow(targetPosition.getY() - currentPose.getY(), 2));
            }
            return Double.NEGATIVE_INFINITY;
        }
    }

    public double getRadiansFromHeadingGoal(){
        synchronized (this) {
            return Math.abs(Localizer.normalizeHeading(currentPose.getTheta() - targetPosition.getTheta()));
        }
    }
    public boolean finished(){
        synchronized (this) {
            return getDistanceFromTarget() < distanceConstraint && getRadiansFromHeadingGoal() < headingConstraint && localizer.getHeadingVelocity()< headingVelocityConstraint && localizer.getXYVelocity()< XYVelocityConstraint;
        }
    }

    public void turnTo(double h){
        synchronized (this) {
            this.lineToConstant(new Pose(currentPose.getX(), currentPose.getY(), h));
        }
    }
    boolean finishedTurning(){
        synchronized (this) {
            return getRadiansFromHeadingGoal() < headingConstraint && localizer.getHeadingVelocity()< headingVelocityConstraint;
        }
    }
    boolean finishedTurning(double radians){
        synchronized (this) {
            return Math.abs(getRadiansFromHeadingGoal()) < radians && localizer.getHeadingVelocity()< headingVelocityConstraint;
        }
    }
    public boolean finished(double inches, double radians){
        synchronized (this) {
            return getDistanceFromTarget() < inches && localizer.getXYVelocity()< XYVelocityConstraint  && Math.abs(getRadiansFromHeadingGoal()) < radians && localizer.getHeadingVelocity()< headingVelocityConstraint;
        }
    }
    public volatile Control control;
    public void setGamepadControl(Gamepad gp){
        this.control= Control.GAMEPAD;
        this.gp= gp;
    }

    public void setAutonomousControl(){
        this.control= Control.AUTO;
    }


    public enum MovementHeading{
        CONSTANT, TANGENTIAL, LINEAR;
    }
    public volatile MovementHeading movementHeading= MovementHeading.CONSTANT;

    public void setStrafeCoefficients(double p, double d){
         pStrafe= p;
        dStrafe= d;
    }
    public void setForwardCoefficients(double p, double d){
        pForward= p;
        dForward = d;
    }
    public void setHeadingCoefficients(double p, double d){
        pHeading= p;
        dHeading= d;
    }

    public Pose getCurrentPosition(){
        return currentPose;
    }

    public void setDistanceConstraint(double constraint){
        distanceConstraint= constraint;
    }

    public Pose getPinpointActualPosition(){
        return localizer.getActualPose();
    }
    public Pose getPinpointRawPosition(){
        return localizer.getRawPosition();
    }

    public void setStartingPosition(Pose p){
        localizer.setStartingPose(p);
        targetPosition= p;
        currentPose= p;
        localizer.resetLocalizer();
    }
    public Pose currentPose= new Pose();
    public Pose targetPosition= new Pose();
    boolean finished= true;
    public void lineToConstant(Pose p){
        synchronized (this) {
            targetPosition = p;
            movementHeading = MovementHeading.CONSTANT;
            finished = false;
        }
    }
    public Pose movementStartPose= new Pose();
    public double totalDistance;
    public void lineToLinear(Pose p){
        synchronized (this) {
            movementStartPose = localizer.getCurrentPosition();
            movementStartPose = localizer.getCurrentPosition();
            targetPosition = p;
            movementHeading = MovementHeading.LINEAR;

            totalDistance = getDistanceFromTarget();
            finished = false;
        }
    }

    public void lineToTangential(Pose p){
        synchronized (this) {
            movementStartPose = localizer.getCurrentPosition();
            targetPosition = p;
            movementHeading = MovementHeading.TANGENTIAL;
            targetHeading = PinpointV1.normalizeHeading(Math.atan((targetPosition.getY() - currentPose.getY()) / (targetPosition.getX() - currentPose.getX())));
            finished = false;
        }
    }
    public void lineToTangential(Pose p, boolean reversed){
        synchronized (this) {
            movementStartPose = localizer.getCurrentPosition();
            targetPosition = p;
            movementHeading = MovementHeading.TANGENTIAL;
            targetHeading = Math.atan((targetPosition.getY() - currentPose.getY()) / (targetPosition.getX() - currentPose.getX()));
            if (reversed)
                targetHeading -= Math.PI;
            finished = false;
        }

    }

    public void setControllers(){
        strafe.setPIDF(pStrafe, 0, dStrafe,0);
        forward.setPIDF(pForward, 0, dForward, 0);
        heading.setPIDF(pHeading, 0, dHeading, 0);
        secondaryStrafe.setPIDF(secondaryPStrafe, 0, secondaryDStrafe,0);
        secondaryForward.setPIDF(secondaryPForward, 0, secondaryDForward, 0);
        secondaryHeading.setPIDF(secondaryPHeading, 0, secondaryDHeading, 0);
    }
    public double theta;
    public double targetHeading= 0;

    public double remainingDistance;

    public  double travelledDistance;
    public double deltaHeading;

    double lateralMultiplier= 1.1;
    public void drive(double s, double f, double r){
        s*= -1;
        r*=-1;
        //  f*=-1;
        r*= lateralMultiplier;
        double d= Math.max((Math.abs(s)+ Math.abs(f)+ Math.abs(r)), 1);

        double fl = (f + s + r) / d;
        double bl = (f - s + r) / d;
        double fr = (f - s - r) / d;
        double br = (f + s - r) / d;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontRight.setPower(fr);



    }
    public double hError;
    public double xRotated, yRotated;
    public void update() {
        synchronized (this) {
            if (control == Control.GAMEPAD) {
                drive(gp.left_stick_x, gp.left_stick_y, gp.right_stick_x);
            } else {
                localizer.update();


                //currentPose = localizer.getCurrentPosition();

                currentPose = new Pose(localizer.getPredictedX(), localizer.getPredictedY(), localizer.getCurrentPosition().getTheta());


                setControllers();

                double x, y;


                //double distanceFromTarget= getDistanceFromTarget();
                if (!useSecondaryStrafe || Math.abs(targetPosition.getY() - currentPose.getY()) > strafeThreshold)
                    y = strafe.calculate(targetPosition.getY(), currentPose.getY());
                else
                    y = secondaryStrafe.calculate(targetPosition.getY(), currentPose.getY());


                if (!useSecondaryForward || Math.abs(targetPosition.getX() - currentPose.getX()) > forwardThreshold)
                    x = forward.calculate(targetPosition.getX(), currentPose.getX());
                else
                    x = secondaryForward.calculate(targetPosition.getX(), currentPose.getX());


                xRotated = x * Math.cos(currentPose.getTheta()) + y * Math.sin(currentPose.getTheta());
                yRotated = x * Math.sin(currentPose.getTheta()) - y * Math.cos(currentPose.getTheta());


                switch (movementHeading) {
                    case LINEAR: {


                        remainingDistance = getDistanceFromTarget();

                        travelledDistance = totalDistance - remainingDistance;
                        deltaHeading = targetPosition.getNormalizedTheta() - movementStartPose.getNormalizedTheta();
                        targetHeading = deltaHeading * (travelledDistance / totalDistance) + movementStartPose.getNormalizedTheta();


                        hError= MathStuff.normalizeRadians(targetHeading- currentPose.getNormalizedTheta());
                        if (!useSecondaryHeading || Math.abs(PinpointV1.normalizeHeading(currentPose.getNormalizedTheta() - targetPosition.getNormalizedTheta())) > headingThreshold)
                            theta = heading.calculate(0, hError);
                        else
                            theta = secondaryHeading.calculate(0, hError);
                        break;
                    }
                    case TANGENTIAL: {

                        double hError= MathStuff.normalizeRadians(targetHeading- currentPose.getNormalizedTheta());
                        if (!useSecondaryHeading || Math.abs(PinpointV1.normalizeHeading(currentPose.getTheta() - targetHeading)) > headingThreshold)
                            theta = heading.calculate(0, hError);
                        else
                            theta = secondaryHeading.calculate(0, hError);

                        break;
                    }
                    case CONSTANT: {

                        targetHeading = targetPosition.getNormalizedTheta();
                        hError= MathStuff.normalizeRadians(targetHeading- currentPose.getNormalizedTheta());
                        if (!useSecondaryHeading || Math.abs(PinpointV1.normalizeHeading(currentPose.getTheta() - targetPosition.getTheta())) > headingThreshold)
                            theta = heading.calculate(0, hError);
                        else
                            theta = secondaryHeading.calculate(0, hError);

                        break;
                    }

                }


                targetHeading = PinpointV1.normalizeHeading(targetHeading);

                drivetrain.updateAuto(yRotated * maxPower, xRotated * maxPower, -theta * maxPower);

            }
        }

    }
    public synchronized void stopChassisThread(){
        running= false;
    }
}

