package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret.FIELD_LENGTH;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.ShooterConstants;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LimelightMath;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.MathStuff;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Vector;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.EvenBetterServo;

@Configurable
public class Shooter {
    public static double velocityThreshold= 50;
    public static DcMotorEx motor1, motor2;
    public static double p = 20, i, d, f = 15;
    public static EvenBetterServo s;
    public static boolean velocitystop = false,shooting = false;
    public static double hoodincrement= 0.045,shootTime=0.12,coefNiggaMan=1.3,waitForNigga=3, hoodtunabil = 0;
    public static double  MAX_RPM = 5800;
    public static double up_pos = 0.2231;
    public static final double g= 9.81;
    public static double hoodangle = 0;
    public static double rpm, pos, testingrpm, ballvelocity = 0,vel;
    public static Vector robotmovementvector=new Vector(0,0);


    public static double rpmThreshhold = 400;
    static PIDFCoefficients shootercoeffs = new PIDFCoefficients(p, i, d, f);
    PIDFController shooterctrl = new PIDFController(p, i, d, f);
    private static double lastX = 0, lastY = 0, lastT = 0;
    private static boolean hasLast = false;

    public static ElapsedTime dt = new ElapsedTime();
    Task t;




    public static double minDist = 1.87, maxDist = 2.46, minINCOutput = 1.5, maxINCOutput = 1.42;
    public static double Power = MathStuff.getLinearFunction(1.58, 2.075, 3100, 3400, Math.hypot(Turret.xCorner, Turret.yCorner));
    public static double HoodPos = MathStuff.getLinearFunction(1.58, 2.075, 0.3392, 0.1114, Math.hypot(Turret.xCorner, Turret.yCorner));

    public static void startMotors() {
        //Blocker.unblock();
        double rpm = LimelightMath.getShooterRPM();
//        m.setMaxRPM(MAX_RPM);
//        m.setRPM(rpm);

    }
    public static void setCoefs(){
        shootercoeffs= new PIDFCoefficients(p,i,d,f);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER ,new PIDFCoefficients(p,i,d,f));
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER ,new PIDFCoefficients(p,i,d,f));
    }
    public static boolean wereCoeffsSet = false;
    public static void setVelocity(double vel){
        if(!wereCoeffsSet)
        {
            wereCoeffsSet = true;
            setCoefs();
        }
        motor1.setVelocity(-vel);
        motor2.setVelocity(-vel);
    }
    public static void stop() {
        velocitystop = true;
        motor1.close();
        //shooter1.setPower(0);
        //shooter2.setPower(0);

    }


    public static double speed,theta;
    public static void updateRobotMovementVector() {
        double xc = Robot.odo.getPosX(DistanceUnit.METER);
        double yc = Robot.odo.getPosY(DistanceUnit.METER);

        double vx = Robot.odo.getVelX(DistanceUnit.METER);
        double vy = Robot.odo.getVelY(DistanceUnit.METER);

        lastX = xc; lastY = yc;

        speed = Math.hypot(vx, vy);
        theta = Math.atan2(vy, vx);
        robotmovementvector = new Vector(speed, theta);
    }


    //    public static double getServoPosition(double angle) {
//        double servopos = (angle + Math.PI / 2) / Math.PI;
//        return Math.max(0.0, Math.min(1.0, servopos));
//    }
    public static double getServoPosition(double angleRad) {
        double minA = ShooterConstants.HOOD_MIN_ANGLE;
        double maxA = ShooterConstants.HOOD_MAX_ANGLE;

        /// clamp unghiul în intervalul mecanic
        angleRad = MathStuff.clamp(angleRad, minA, maxA);

        /// normalize 0..1
        //  double t = (angleRad - minA) / (maxA - minA);
        double t = MathStuff.getLinearFunction(ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE, ShooterConstants.HOOD_MIN_POS, ShooterConstants.HOOD_MAX_POS, angleRad);

        //t = 1.0 - t;

        return Range.clip(t, 0.0, 1.0);
    }




    public static void setResolution(double res) {
       // m.setTicksPerRevolution(res);

    }

    public static double incrementaldistance= 3;
    public static double changeIncremental(){
        if(Math.hypot(Turret.xCorner, Turret.yCorner) >incrementaldistance){
            inc = 1.57;
        }
        else inc = MathStuff.getLinearFunction(minDist,maxDist,minINCOutput,maxINCOutput,Math.hypot(Turret.xCorner, Turret.yCorner));
        return inc;
    }
    public static double transformForMotor(double res){
        double wheelrpm=(res / (2*Math.PI*0.036)) * 60.0;
        return (wheelrpm/60.0)*28;
    }

    public static double transformForServo(double res){
        return MathStuff.clamp(Math.abs(ShooterConstants.HOOD_MIN_ANGLE-ShooterConstants.HOOD_MAX_POS)/(Math.abs(ShooterConstants.HOOD_MIN_ANGLE-ShooterConstants.HOOD_MAX_ANGLE))*(res-ShooterConstants.HOOD_MIN_ANGLE)+ShooterConstants.HOOD_MIN_POS, 0.0, 1.0 );
    }
    public static void setVelocityCalculated(){
        motor1.setVelocity(transformForMotor(ballvelocity));
        s.setPosition(getServoPosition(hoodangle));
    }


    public static double getPower() {
        return motor1.getPower();
    }


    public static double getRPM() {
       // return m.getRPM();
        return 0;
    }

    public static void changePIDF(double p, double i, double d, double f)
    {
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER ,new PIDFCoefficients(p,i,d,f));
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER ,new PIDFCoefficients(p,i,d,f));
    }
    public static void setHood2() {
        s.setPosition(hoodtunabil);
    }

    public static void setHood() {
        double angle = LimelightMath.getLimelightAngle();
        //convert
    }


    public static double coordinateTheta,parralelvelocity,launchVelocityForY,newXVelocity,compensatedVelocitY, turretOffset;
    public static double distt, tangentialvelocity = 0, time;
    public static Vector robotVelocity= new Vector(0,0);
    public static final double a1 = -0.0641147, b1 = 0.390703, c1 = 0.18375;
    public static final double a2 = -100.04613, b2 = 804.30749, c2 = 357.66532;
    public static double inc=1;
    public static void calculateShooterVelocity(double robotheading) {
        double x = Math.hypot(Turret.xCorner, Turret.yCorner)- ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;


        hoodangle = MathStuff.clamp(Math.atan(2 * y / x - Math.tan(a)), ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);//initial angle
        ballvelocity = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodangle), 2) * (x * Math.tan(hoodangle) - y)));

        robotVelocity = new Vector(robotmovementvector.getMagnitude(), robotheading);


        coordinateTheta =robotVelocity.getTheta() - robotmovementvector.getTheta();

        parralelvelocity = Math.cos(coordinateTheta) *robotmovementvector.getMagnitude();
        tangentialvelocity = Math.sin(coordinateTheta) *robotmovementvector.getMagnitude();

        time = x /( ballvelocity * Math.cos(hoodangle));
        launchVelocityForY = ballvelocity * Math.sin(hoodangle);
        compensatedVelocitY =( x/ time) + parralelvelocity;
        newXVelocity = Math.sqrt(Math.pow(compensatedVelocitY, 2) + Math.pow(tangentialvelocity, 2));
        x = newXVelocity * time;

        hoodangle = Math.atan(launchVelocityForY / newXVelocity);
        if (Double.isNaN(hoodangle) || Double.isInfinite(hoodangle)) hoodangle=ShooterConstants.HOOD_MIN_ANGLE;
        ballvelocity = Math.sqrt(g * x * x / ( Math.pow(Math.cos(hoodangle), 2) * (x  * Math.tan(hoodangle) - y)));
        ballvelocity=ballvelocity*changeIncremental();
    }



    public static void set(){
        velocitystop=false;
        distt = Turret.dist;
        if(distt < 1.65) {
            HoodPos = 0.63;
            vel = 1400;
        }
        else if(distt>3.28)
        {
            HoodPos = 0.75;
            vel =2000;
        }
        else {
            HoodPos = Range.clip(a1 * Math.pow(distt, 2) + b1 * distt + c1, 0.58, 0.789);
            vel = Math.pow(distt,2)*a2+b2*distt+c2;
        }
    }
    public static void setPow(){
        Power= MathStuff.getLinearFunction(1.62, 2.88, 3600, 4500, Math.hypot(Turret.xCorner,Turret.yCorner));
        setRPM(Power);
    }
    public static void set2(double RPM, double pos){
        setRPM(RPM); //testing
        s.setPosition(pos);
    }

    public static void getLimelightDistance() {

    }
    public static final double m1= - 0.264334, n1= 1.3949, p1= -1.53374- 0.125;
    public static final double m2= - 1.41377, n2= 2.52986, p2= 0;
    public static double y, x, dist;
    public static double[] get(){
        x = Robot.odo.getPosX(DistanceUnit.METER);
        y = Robot.odo.getPosY(DistanceUnit.METER);
        y = FIELD_LENGTH / 2 + y;
        x = Robot.a == Alliance.RED ? FIELD_LENGTH / 2 + x : FIELD_LENGTH / 2 - x;

        dist = Math.hypot(x, y);
        if( y> 2.4 || dist> 3.3){
            rpm = 6000;
            pos= up_pos;
        }
        else if(dist> 1.5){
            rpm= 4800;
            pos= m1 * dist * dist + n1 * dist + p1;
        }
        else{
            rpm= 3600;
            pos= m2 * dist * dist + n2 * dist + p2;
        }
        return new double[]{dist, pos};

    }

    public static void simpleSet(){
        motor1.setPower(1);
    }

    public static void setMT1(){
       motor1.setPower(1);
    }
    public static void setMT2(){
        motor1.setPower(-1);
    }
    public static void setRPM(double rpm){
        Shooter.rpm = rpm;
    }


    public static void startMotorsSimple(){
        motor1.setPower(1);
    }
    public static boolean RPM_Reached(){
        //return Math.abs(rpm-getRPM())<rpmThreshhold;
        return true;
    }
    public static boolean hoodFinished(){
        return s.finished();
    }

    public static boolean readyToGo(){
        return !motor1.isBusy() && s.finished();
    }
    public static void waitTheTurn(){
        if(LimelightMath.getShooterRPM() == MAX_RPM )
        {
            //Blocker.unblock();

        }
    }
    double minPos= .508, maxPos= .8;




    public static double targetvel1, targetvel2;


    public static void update(){
        set();
    }
    public static void write(){
        if(!shooting)
            s.setPosition(HoodPos);
        setVelocity(vel);
    }


}
