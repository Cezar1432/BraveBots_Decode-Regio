package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LimelightMath;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotorEx;

@Configurable
public class Turret {
    public static BetterCRServo s1, s2;
    public static BetterMotorEx m;
    public static double p = 0.02, d = 0.0006, i = 0.05, f = 0, st = 0;
    private static double  lastAngle;
    public static double GEAR_RATIO= 1.05;
    public static double CAMERA_RESOLUTION= 640;
    public static final double RANGE= 302 , NEUTRAL_POSITION= 0.003;
    public static double MAX_ANGLE = RANGE / 2.0 - RANGE * NEUTRAL_POSITION, MIN_ANGLE = -RANGE / 2.0 + RANGE * NEUTRAL_POSITION, MAX_POS = .5 + 120 / 177.5, MIN_POS = 1 - 120. / 177.5;
    public static final double FIELD_LENGTH = 3.65;
    public static final double errorThreshold= 3,angleThreshold= 1;
    public static double maxTicks= 100;
    public static double angle;
    public static double finalAngle= 0;
    PIDFCoefficients coefs= new PIDFCoefficients(p,i,d,f);
    public static double dist= 0;
    public volatile static boolean running= false;
    public static void startTurretThread(){
        running= true;
        //Robot.pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        new Thread(()->{
            while (running && !Thread.interrupted()){
                try {
                    update();
                    Thread.sleep(1);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }
        }).start();
    }
    public static void stopTurretThread(){
        running= false;
    }
    public static PIDFController c= new PIDFController(p,i,d,0);

    public enum Action{
        TRACK, ADJUST, BACK_TO_NEUTRAL;
    }

    public static void setPosition(double position){
        s1.setPosition(position);
        s2.setPosition(position);
    }
    static double normalizeNothing(double a){
        while (a> GEAR_RATIO/ 2) a-=GEAR_RATIO;
        while (a< -GEAR_RATIO/ 2) a+=GEAR_RATIO;
        return a;
    }
    public static double getPosition(){
        return s1.getTruePosition();
    }
    public static Action action= Action.TRACK;
    public static void adjustToCorner(){
        double angleDiff = LimelightMath.getTurretAngle() - LimelightMath.getLimelightAngle();
        /*
        s.setFunnyTarget(s.get  angleDiff / 360 * GEAR_RATIO);
        double angle= normalize(angleDiff+ s.getTotalPosition());
        s.setFunnyTarget(angle);
        action= Action.ADJUST;
        */
    }

    public enum States{
        UPDATE, RESET
    }
    static Robot robot= Robot.getInstance();
    public static States state= States.RESET;
    public static ElapsedTime timer= new ElapsedTime();
    public static ElapsedTime updateTimer= new ElapsedTime();

    public static void reset(){
        state= States.RESET;
    }
    public static void changeCoeffs(){
        c= new PIDFController(p,i,d,0);
    }

    //    public static double getTurretAngle(){
//        return (s.getPosition()- .35 - NEUTRAL_POSITION)  * MAX_ANGLE  * 2;
//    }
    static boolean alignedByLimelight= false;

    public static double fieldRelative, robotRelative, turretRelative;
    public static double x, y, xCorner, yCorner;
    public static final double UPDATE_PERIOD= 700, PINPOINT_UPDATE_PERIOD = 2000;
    public static volatile boolean conditioneAllign= false;
    public static double position;
    public static double normalize(double h){
        while (h> 180) h-=360;
        while (h< -180) h+= 360;
        return h;
    }

    public synchronized static void update(){
        /*
        if(s.getTotalPosition()< GEAR_RATIO* MAX_POSSIBLE_REVS + NEUTRAL_POSITION || s.getTotalPosition()> -(GEAR_RATIO* MAX_POSSIBLE_REVS) + NEUTRAL_POSITION) {
            action = Action.BACK_TO_NEUTRAL;
            s.setFunnyTarget(NEUTRAL_POSITION);
            s.changeThreshold(0.4);
        }
        track();
        if(action== Action.BACK_TO_NEUTRAL && s.finished2()){
            s.changeThreshold(0.05);
            action= Action.TRACK;
        }
        */
//        if(updateTimer.milliseconds()> PINPOINT_UPDATE_PERIOD){
//        Pose p = LimelightMath.getRobotPose();
//        if (p != null && !(p.x == 0 || p.y == 0)) {
//            Robot.pp.setPosX(p.getX(), DistanceUnit.METER);
//            Robot.pp.setPosY(p.getY(), DistanceUnit.METER);
//            alignedByLimelight = true;
//            updateTimer.reset();
//            // Robot.pp.setPosition(new Pose2D(0, 0, 0, 0 , 0));
//        } else {
//            alignedByLimelight = false;
//        }
//        }
//        double x = Robot.pp.getPosX(DistanceUnit.INCH);
//        double y = Robot.pp.getPosY(DistanceUnit.INCH);
//        if(Robot.alliance == Robot.Alliance.RED) {
//            x = 144 + x;
//            y = 144 - y;
//        }
//        else
//        {
//            x= 144-x;
//            y= 144-y;
//        }
//        double alpha = Math.atan(y / x);
//        double heading = Robot.pp.getHeading(AngleUnit.DEGREES);
//        if(heading < 0 ) heading += 360;
//        if(Robot.alliance == Robot.Alliance.RED)
//            robotRelative = 180 - heading + Math.toDegrees(alpha);

        /*
        try {
            Robot.pp.setPosY(p.getX(), DistanceUnit.METER);
            Robot.pp.setPosX(p.getY(),DistanceUnit.METER);
            alignedByLimelight= true;
        }
        catch (NullPointerException e) {
            alignedByLimelight= false;
        }
        */
//        x = Robot.pp.getPosX(DistanceUnit.METER);
//        y = Robot.pp.getPosY(DistanceUnit.METER);
//        y = FIELD_LENGTH / 2 + y;
//        x = Robot.alliance == Robot.Alliance.RED ? FIELD_LENGTH / 2 + x : FIELD_LENGTH / 2 - x;
//        fieldRelative = Math.toDegrees(Math.atan(y/x));
//        if (Robot.alliance == Robot.Alliance.BLUE)
//            fieldRelative = 180 - fieldRelative;
//
//        robotRelative = Robot.pp.getHeading(AngleUnit.DEGREES) - fieldRelative;
//        robotRelative = Range.clip(robotRelative, MIN_ANGLE, MAX_ANGLE);
//        position= robotRelative/ RANGE + 0.5 + NEUTRAL_POSITION;
        // Pose p= LimelightMath.getRobotPose();

/// /////////////////////////////

        if (robot.odo.getPosX(DistanceUnit.METER) != 0 && robot.odo.getPosY(DistanceUnit.METER) != 0) {
            x = robot.odo.getPosX(DistanceUnit.METER);
            y = robot.odo.getPosY(DistanceUnit.METER);

            yCorner = FIELD_LENGTH- y;
            xCorner = robot.a == Alliance.RED ? FIELD_LENGTH - x : FIELD_LENGTH + x;

            dist= Math.hypot(xCorner, yCorner);
            fieldRelative = Math.atan(yCorner / xCorner);
            fieldRelative = Math.toDegrees(fieldRelative);
            if(robot.a== Alliance.BLUE)
                fieldRelative= 180- fieldRelative;
            robotRelative= normalize(fieldRelative- robot.odo.getHeading(AngleUnit.DEGREES));
            turretRelative= normalize(robotRelative- 180);

            robotRelative = robot.a == Alliance.RED ? robotRelative : -robotRelative;


            // if (timer.milliseconds() > UPDATE_PERIOD && Math.abs(lastAngle - robotRelative) > angleThreshold) {
            setAngle(turretRelative);


            c.setPIDF(p,i,d,0);
            double power= c.calculate(getAngle(), finalAngle);
            s1.setPower(-power);
            s2.setPower(power);


        }



    }
    public static double getAngle(){
        return (1-(s1.getTruePosition()- NEUTRAL_POSITION))* 360 * GEAR_RATIO - 180* GEAR_RATIO;
    }
    public static void setAngle(double angle){
        //angle= Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        if(angle< MIN_ANGLE)
            angle= MIN_ANGLE;
        if(angle> MAX_ANGLE)
            angle= MAX_ANGLE;

        finalAngle= angle;
        // position= Math.abs(1-(finalAngle / RANGE * 1.05 + 0.493 + NEUTRAL_POSITION))+ 0.0198;

//        position = Range.clip(position, 0.08 , 0.92);
//        setPosition(position);
    }



//    public static boolean finished(){
//       return Math.abs(Math.abs(robotRelative)- Math.abs(getTurretAngle()))< errorThreshold;
//    }


}
