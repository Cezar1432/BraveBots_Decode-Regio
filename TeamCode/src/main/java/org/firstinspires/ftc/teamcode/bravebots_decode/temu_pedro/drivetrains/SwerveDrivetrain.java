package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.slew_rate_limiter.GamepadLimiter;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;


public class SwerveDrivetrain implements DrivetrainInterface {

    BetterCRServo sFL, sFR, sBL, sBR;
    BetterMotor mFL, mFR, mBL, mBR;

    public SwerveModule fl, fr, bl, br;

    Robot robot;
    public SwerveDrivetrain(Robot robot) {

        this.robot= robot;
        initialize(robot);
        fr = new SwerveModule(mFR, sFR);
        br = new SwerveModule(mBR, sBR);
        fl = new SwerveModule(mFL, sFL);
        bl = new SwerveModule(mBL, sBL);
    }



    public void initialize(Robot robot) {
        sFL = robot.fl;
        sBL = robot.bl;
        sBR = robot.br;
        sFR = robot.fr;
        mFL = robot.leftFront;
        mBL = robot.leftBack;
        mBR = robot.rightBack;
        mFR = robot.rightFront;
        mBL.setDirection(DcMotorSimple.Direction.REVERSE);

        mFL.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    volatile Gamepad gp;
    volatile GamepadLimiter gamepadLimiter;
    public SwerveDrivetrain(Robot robot, Gamepad gp, double limiter){
        this.robot= robot;
        initialize(robot);
        fr = new SwerveModule(mFR, sFR);
        br = new SwerveModule(mBR, sBR);
        fl = new SwerveModule(mFL, sFL);
        bl = new SwerveModule(mBL, sBL);
        this.gp= gp;
        gamepadLimiter= new GamepadLimiter(this.gp, limiter);

    }

    private double wheelBase = 0;
    private double trackWidth = 0;
    //private  double r = Math.sqrt((wheelBase * wheelBase) + (trackWidth * trackWidth));


    public double radius= hypot(wheelBase, trackWidth);
    public void setWheelBase(double wheelBase){
        this.wheelBase= wheelBase;
    }
    public void setTrackWidth(double trackWidth){
        this.trackWidth= trackWidth;
    }


    public class SwerveModule {
        public BetterMotor driveMotor;
        public BetterCRServo steeringServo;

        double targetSpeed = 0;
        double targetAngle = 0;

        public SwerveModule(BetterMotor drive, BetterCRServo rotation) {
            this.driveMotor = drive;
            this.steeringServo = rotation;

        }



        double speed;
        public synchronized void setState(double speed, double angle) {
            // Get current angle
            double currentAngle = steeringServo.getTruePosition() * 360.0;

            // Calculate shortest path
            double diff = angle - currentAngle;
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;

            if (Math.abs(diff) > 90) {
                diff = diff > 0 ? diff - 180 : diff + 180;
                speed = -speed;
            }

            double targetAngle = currentAngle + diff;


            steeringServo.setAngle(targetAngle);
            steeringServo.update();

            double alignmentFactor=  1 - Math.abs(diff)/90;

            driveMotor.setPower(speed * alignmentFactor);
            this.speed= speed;
        }


        public double getCurrentAngle() {
            return steeringServo.getTruePosition() * 360.0;
        }


        public double getTargetAngle() {
            return steeringServo.getTargetPos() * 360.0;
        }
        public double getTargetPower(){
            return this.speed;
        }

        public double getDriveSpeed() {
            return driveMotor.getPower();
        }

        public void stop(double angle) {
            driveMotor.setPower(0);
            steeringServo.setAngle(angle);
        }


        public void setCoefs(PDSFCoefficients coefs) {
            steeringServo.setCoefs(coefs.p, coefs.d, coefs.s, coefs.f);
        }
    }
    public boolean ok= false;
    PIDFController headingController;

    double lastFRangle= 0, lastFLangle= 0, lastBRangle= 0, lastBLangle= 0;
    volatile boolean running= false;
    volatile double lastAngle;
    public void startUpdateThread(){
        Robot r= Robot.getInstance();
        headingController= new PIDFController(Constants.strafe.getP(), Constants.strafe.getI(), Constants.strafe.getD(), Constants.strafe.getF());
        Thread t= new Thread(()->{
            while(running && Thread.currentThread().isAlive()) {
                try {
                    double strafeX = gamepadLimiter.getLeftX();
                    double strafeY = gamepadLimiter.getLeftY();
                    double rotation = gamepadLimiter.getRightX();
                    if (trackWidth == 0)
                        throw new IllegalArgumentException("Track Width nesetat");
                    if (wheelBase == 0)
                        throw new IllegalArgumentException("Wheel Base nesetat");
                    if (Math.abs(strafeX) > 0.02 || Math.abs(strafeY) > 0.02 || Math.abs(rotation) > 0.02) {
                        radius = hypot(wheelBase, trackWidth);
                        if(rotation< .1)
                            rotation= headingController.calculate(r.odo.getHeading(AngleUnit.DEGREES), lastAngle);
                        else {
                            rotation *= -1.1;
                            lastAngle= r.odo.getHeading(AngleUnit.DEGREES);
                        }


                        //strafeY *= -1;
                        // strafeX*=-1;
                        double a = strafeX + rotation * (wheelBase / radius),
                                b = strafeX - rotation * (wheelBase / radius),
                                c = strafeY + rotation * (trackWidth / radius),
                                d = strafeY - rotation * (trackWidth / radius);


                        double flSpeed = hypot(b, c),
                                frSpeed = hypot(b, d),
                                brSpeed = hypot(a, d),
                                blSpeed = hypot(a, c);
                        double flAngle = atan2(b, c),
                                frAngle = atan2(b, d),
                                brAngle = atan2(a, d),
                                blAngle = atan2(a, c);

                        double max = Math.max(Math.max(flSpeed, frSpeed), Math.max(brSpeed, blSpeed));
                        if (max > 1) {
                            flSpeed /= max;
                            frSpeed /= max;
                            blSpeed /= max;
                            brSpeed /= max;
                        }

                        fl.setState(flSpeed, toDegrees(flAngle));
                        fr.setState(frSpeed, toDegrees(frAngle));
                        bl.setState(brSpeed, toDegrees(blAngle));
                        br.setState(blSpeed, toDegrees(brAngle));

                        lastFLangle = frAngle;
                        lastFRangle = flAngle;
                        lastBRangle = blAngle;
                        lastBLangle = brAngle;
                    } else {
                        fl.setState(0, toDegrees(lastFLangle));
                        fr.setState(0, toDegrees(lastFRangle));
                        bl.setState(0, toDegrees(lastBLangle));
                        br.setState(0, toDegrees(lastBRangle));
                    }
                    Thread.sleep(1);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }

        });
        running= true;

    }
    public double rot;

    @Override
    public void update(double strafeX, double strafeY, double rotation) {

        if(headingController== null){
            headingController= new PIDFController(Constants.strafe.getP() * 1.2, Constants.strafe.getI(), Constants.strafe.getD() * 1.2, Constants.strafe.getF());
        }
        if(trackWidth== 0)
            throw new IllegalArgumentException("Track Width nesetat");
        if(wheelBase== 0)
            throw new IllegalArgumentException("Wheel Base nesetat");
        if(Math.abs(strafeX) > 0.02 || Math.abs(strafeY)> 0.02 || Math.abs(rotation)> 0.02) {
            radius= hypot(wheelBase, trackWidth);
            //double heading= robot.odo.getHeading(AngleUnit.DEGREES);

            //strafeY *= -1;
            // strafeX*=-1;
            if(Math.abs(rotation)>.1){
                rotation*= -1.15;
                lastAngle= robot.robotHeading;
            }
            else{
                rotation= headingController.calculate(robot.robotHeading, lastAngle);
            }
            rot= rotation;
            double a = strafeX + rotation * (wheelBase / radius),
                    b = strafeX - rotation * (wheelBase / radius),
                    c = strafeY + rotation * (trackWidth / radius),
                    d = strafeY - rotation * (trackWidth / radius);


            double flSpeed = hypot(b, c),
                    frSpeed = hypot(b, d),
                    brSpeed = hypot(a, d),
                    blSpeed = hypot(a, c);
            double flAngle = atan2(b, c),
                    frAngle = atan2(b, d),
                    brAngle = atan2(a, d),
                    blAngle = atan2(a, c);

            double max = Math.max(Math.max(flSpeed, frSpeed), Math.max(brSpeed, blSpeed));
            if (max > 1) {
                flSpeed /= max;
                frSpeed /= max;
                blSpeed /= max;
                brSpeed /= max;
            }

            fl.setState(flSpeed, toDegrees(flAngle));
            fr.setState(frSpeed, toDegrees(frAngle));
            bl.setState(brSpeed, toDegrees(blAngle));
            br.setState(blSpeed, toDegrees(brAngle));

            lastFLangle = flAngle;
            lastFRangle = frAngle;
            lastBLangle = blAngle;
            lastBRangle = brAngle;
        }

        else{
            fl.setState(0, toDegrees(lastFLangle));
            fr.setState(0, toDegrees(lastFRangle));
            bl.setState(0, toDegrees(lastBLangle));
            br.setState(0, toDegrees(lastBRangle));
        }



    }
    @Override
    public void updateAuto(double strafeX, double strafeY, double rotation) {
//         Calculate wheel vectors
//         For rotation: each wheel moves perpendicular to its position vector
//         FL is at (-trackWidth/2, wheelBase/2), rotation adds (wheelBase/2, trackWidth/2) to velocity
//         FR is at (trackWidth/2, wheelBase/2), rotation adds (wheelBase/2, -trackWidth/2) to velocity
//         BL is at (-trackWidth/2, -wheelBase/2), rotation adds (-wheelBase/2, trackWidth/2) to velocity
//         BR is at (trackWidth/2, -wheelBase/2), rotation adds (-wheelBase/2, -trackWidth/2) to velocity
        double smth= Math.sqrt(strafeX* strafeX + strafeY * strafeY);
        double smth2= -.5 * smth+ .75;
        rotation= rotation* smth2;
        if(Math.abs(strafeX) > 0.08 || Math.abs(strafeY)> 0.08 || Math.abs(rotation)> 0.08) {
            rotation *= -1;
            strafeX *= -1;
            strafeY*= -1;
            double a = strafeX + rotation * (wheelBase / radius),
                    b = strafeX - rotation * (wheelBase / radius),
                    c = strafeY + rotation * (trackWidth / radius),
                    d = strafeY - rotation * (trackWidth / radius);

            double flSpeed = hypot(b, c),
                    frSpeed = hypot(b, d),
                    brSpeed = hypot(a, d),
                    blSpeed = hypot(a, c);
            double flAngle = atan2(b, c),
                    frAngle = atan2(b, d),
                    brAngle = atan2(a, d),
                    blAngle = atan2(a, c);

            double max = Math.max(Math.max(flSpeed, frSpeed), Math.max(brSpeed, blSpeed));
            if (max > 1) {
                flSpeed /= max;
                frSpeed /= max;
                blSpeed /= max;
                brSpeed /= max;
            }

            fl.setState(frSpeed, toDegrees(frAngle));
            fr.setState(flSpeed, toDegrees(flAngle));
            bl.setState(brSpeed, toDegrees(brAngle));
            br.setState(blSpeed, toDegrees(blAngle));

            lastFLangle = frAngle;
            lastFRangle = flAngle;
            lastBRangle = brAngle;
            lastBLangle = blAngle;
            ok= true;
        }

        else{
            fl.setState(0, toDegrees(lastFLangle));
            fr.setState(0, toDegrees(lastFRangle));
            bl.setState(0, toDegrees(lastBLangle));
            br.setState(0, toDegrees(lastBRangle));
            ok= false;
        }



    }

    @Override


    public void setCoefs(PDSFCoefficients c){
        fl.setCoefs(c);
        fr.setCoefs(c);
        bl.setCoefs(c);
        br.setCoefs(c);
    }


}


