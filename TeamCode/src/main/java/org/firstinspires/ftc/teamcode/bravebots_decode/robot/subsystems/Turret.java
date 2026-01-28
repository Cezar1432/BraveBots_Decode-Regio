package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LimelightMath;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotorEx;

import java.util.List;

public class Turret {


    public static double filterParameter= 1;
    static LowPassFilter filter= new LowPassFilter(0, filterParameter);
    public static BetterMotor m;
    //public static double p = 0.02, d = 0.0006, i = 0.05, f = 0, st = 0, s = 0,p2=0,d2=0,i2=0,f2=0;
    //public static double p = 0.009, d = 0.0007, i = 0, f = 0, st = 0, s = 0.065,p2=0.03,d2=0.00004,i2=0,f2=0;
    //public static double p = 0.01, d = 0.0002, i = 0, f = 0, st = 0, s = 0.055,p2=0.02,d2=0.0001,i2=0,f2=0;
    public static double p = 0.00012, d = 0.000003, i = 0.02, f = 0, st = 0, s = 0.055,p2=0.0115,d2=0.0001,i2=0,f2=0;
    private static double  lastAngle;
    public static double GEAR_RATIO= 1.05;
    public static double CAMERA_RESOLUTION= 640;
    public static final double RANGE= 302 , NEUTRAL_POSITION= 0.003;
    public static double MAX_ANGLE = RANGE / 2.0 - RANGE * NEUTRAL_POSITION, MIN_ANGLE = -RANGE / 2.0 + RANGE * NEUTRAL_POSITION, MAX_POS = .5 + 120 / 177.5, MIN_POS = 1 - 120. / 177.5;
    public static final double FIELD_LENGTH = 3.65;
    public static double errorThreshold= 3,angleThreshold= 5;

    public volatile static double angle,power,sign,kSOutput;
    public static double finalAngle= 0;
    PDSFCoefficients coefs2 = new PDSFCoefficients(p,d,s,f);
    PIDFCoefficients coefs= new PIDFCoefficients(p,i,d,f);
    public volatile static boolean running= false,running2= false, allignedbylimelight = false;
    public static void startTurretThread(){
        running= true;
        //Robot.pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        new Thread(()->{
            while (running && !Thread.interrupted()){
                try {
                    update();
                    Thread.sleep(8);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }
        }).start();
    }
    public static void startTurretThreadForAuto(){
        running2= true;
        //Robot.pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        new Thread(()->{
            while (running2 && !Thread.interrupted()){
                try {
                    updateForAuto();
                    Thread.sleep(4);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }
        }).start();
    }
    public static void stopTurretThread(){
        running= false;running2=false;
    }
    public volatile static PIDFController c= new PIDFController(p,i,d,f);

    public static void setCoeffs(double p, double i, double d, double f){
        c.setPIDF(p, i, d, f);
    }
    public static PIDFController c2 = new PIDFController(p2,i2,d2,f2);

    public enum Action{
        TRACK, ADJUST, BACK_TO_NEUTRAL;
    }

    static double normalizeNothing(double a){
        while (a> GEAR_RATIO/ 2) a-=GEAR_RATIO;
        while (a< -GEAR_RATIO/ 2) a+=GEAR_RATIO;
        return a;
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
    public static States state= States.RESET;
    public static ElapsedTime timer= new ElapsedTime();
    public static ElapsedTime updateTimer= new ElapsedTime();

    public static void reset(){
        m.resetEncoder();
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
    public static Position positionll;
    public static Pose3D pose;


    public static double normalize(double h){
        while (h> 180) h-=360;
        while (h< -180) h+= 360;
        return h;
    }

    public volatile static double dist;
    public synchronized static void updateForAuto(){
        if (Robot.odo.getPosX(DistanceUnit.METER) != 0 && Robot.odo.getPosY(DistanceUnit.METER) != 0) {
            x = Robot.odo.getPosX(DistanceUnit.METER);
            y = Robot.odo.getPosY(DistanceUnit.METER);

            yCorner = FIELD_LENGTH - y;
            xCorner = Robot.a == Alliance.RED ? FIELD_LENGTH - x : FIELD_LENGTH + x;

            fieldRelative = Math.atan(yCorner / xCorner);
            fieldRelative = Math.toDegrees(fieldRelative);
            if (Robot.a == Alliance.BLUE)
                fieldRelative = 180 - fieldRelative;
            robotRelative = normalize(fieldRelative - Robot.odo.getHeading(AngleUnit.DEGREES));
            turretRelative = normalize(180 - robotRelative);

            robotRelative = Robot.a == Alliance.RED ? robotRelative : -robotRelative;



            // if (timer.milliseconds() > UPDATE_PERIOD && Math.abs(lastAngle - robotRelative) > angleThreshold) {
            setAngle(turretRelative);
//            c.setPIDF(p, i, d, f);
//            c2.setPIDF(p2, i2, d2, f2);
//            sign = Math.signum(finalAngle - getAngle());
//            kSOutput = s * sign;
//            if (Math.abs(finalAngle - getAngle()) > angleThreshold)
//                power = c.calculate(getAngle(), finalAngle) + kSOutput;
//
//            else
//                power = c2.calculate(getAngle(), finalAngle) + kSOutput;
//            double voltage= Robot.voltageSensor.getVoltage();
//            if(voltage< 12.0)
//                power*= (12.0/ voltage);
//            s1.setPower(-power);
//            s2.setPower(power);
        }
    }
    public static final double /*ticksPerRevolution= 24272.6,*/ ticksPerRevolution=40960, ticksPerDegree= ticksPerRevolution/ 360.0;
    public static double targetTicks= 0;
    public static double minTicks= -ticksPerRevolution/2, maxTicks= ticksPerRevolution/2;
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


        try {
            LLResult res = LimelightMath.getResults();
            if (res != null && res.isValid()) {
               List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();
               for(LLResultTypes.FiducialResult fiduci : fiducials)
               {
                   int apriltagID = fiduci.getFiducialId();
                   if((Robot.a == Alliance.RED && apriltagID == 24) || (Robot.a == Alliance.BLUE && apriltagID ==20))
                   {
                       allignedbylimelight = true;
                       pose = res.getBotpose_MT2();
                       positionll = pose.getPosition();
                       if (positionll.x != 0 && positionll.y != 0) {
                           Robot.odo.setPosX(positionll.y, DistanceUnit.METER);
                           Robot.odo.setPosY(-positionll.x, DistanceUnit.METER);
                       }
                   }
               }
            }
        }
        catch (Throwable e){
            android.util.Log.e("Turrret", e.getMessage(), e);
        }

        if (Robot.odo.getPosX(DistanceUnit.METER) != 0 && Robot.odo.getPosY(DistanceUnit.METER) != 0) {
            y = Robot.odo.getPosX(DistanceUnit.METER);
            x = Robot.odo.getPosY(DistanceUnit.METER);

            yCorner = Robot.a== Alliance.RED ? FIELD_LENGTH/2- y: FIELD_LENGTH/2+ y;
            xCorner = FIELD_LENGTH/2 - x ;

            dist= Math.hypot(xCorner, yCorner);

            fieldRelative = Math.atan(xCorner / yCorner);
            fieldRelative = Math.toDegrees(fieldRelative);
            if(Robot.a== Alliance.BLUE)
                fieldRelative= 180- fieldRelative;
            robotRelative= normalize(fieldRelative- Robot.odo.getHeading(AngleUnit.DEGREES));
            turretRelative= normalize(robotRelative- 180);

            robotRelative = Robot.a == Alliance.RED ? robotRelative : -robotRelative;
            targetTicks= ticksPerDegree * turretRelative;
            targetTicks= Range.clip(targetTicks, minTicks, maxTicks);
            c.setPIDF(p, i, d, f);


            // if (timer.milliseconds() > UPDATE_PERIOD && Math.abs(lastAngle - robotRelative) > angleThreshold) {
            //setAngle(turretRelative);
//            c.setPIDF(p,i,d,f);
//            c2.setPIDF(p2,i2,d2,f2);
//            sign=Math.signum(finalAngle-getAngle());
//            kSOutput = s*sign;
//            if(Math.abs(finalAngle-getAngle())>angleThreshold)
//                power=c.calculate(getAngle(), finalAngle)+kSOutput;
//            else
//                power=c2.calculate(getAngle(),finalAngle)+kSOutput;
//            double voltage= Robot.voltageSensor.getVoltage();
//            if(voltage< 12.0)
//                power*= (12.0/ voltage);
//            s1.setPower(-power);
//            s2.setPower(power);
        }



    }
    public static void write(){
        power = c.calculate(m.getCurrentPosition(), targetTicks);
        m.setPower(power);
    }
//    public static double getAngle(){
//        return (1-(s1.getTruePosition()- NEUTRAL_POSITION))* 360 * GEAR_RATIO - 180* GEAR_RATIO;

    public static void setAngle(double angle){
        //angle= Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        if(angle< MIN_ANGLE)
            angle= MIN_ANGLE;
        if(angle> MAX_ANGLE)
            angle= MAX_ANGLE;


    }
    public static double getTicks(){
        return m.getCurrentPosition();
    }
    public static double getAngle(){
        return getTicks()/ ticksPerDegree;
    }



//    public static boolean finished(){
//       return Math.abs(Math.abs(robotRelative)- Math.abs(getTurretAngle()))< errorThreshold;
//    }



}