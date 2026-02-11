package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.MathStuff;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.slew_rate_limiter.GamepadLimiter;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;
import org.opencv.core.Mat;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Configurable

public class SwerveDrivetrain implements DrivetrainInterface {

    BetterCRServo sFL, sFR, sBL, sBR;
    BetterMotor mFL, mFR, mBL, mBR;

    public SwerveModule fl, fr, bl, br;

    Robot robot;
    boolean headingLock;
    double headingCorrection;
    public SwerveDrivetrain(Robot robot) {

        this.robot= robot;
        initialize(robot);
        fr = new SwerveModule(mFR, sFR);
        br = new SwerveModule(mBR, sBR);
        fl = new SwerveModule(mFL, sFL);
        bl = new SwerveModule(mBL, sBL);
        headingLock= false;
        headingCorrection= 0;
        headingController= new PIDFController(0.01 , Constants.strafe.getI(), 0.001, Constants.strafe.getF());

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

    public static double leftFrontOffset= -15, rightFrontOffset= -5, leftBackOffset = -5, rightBackOffset= -8;
    private double wheelBase = 0;
    private double trackWidth = 0;
    //private  double r = Math.sqrt((wheelBase * wheelBase) + (trackWidth * trackWidth));


    public double radius= hypot(wheelBase, trackWidth);
    public SwerveDrivetrain setWheelBase(double wheelBase){
        this.wheelBase= wheelBase;
        return this;
    }
    public SwerveDrivetrain setTrackWidth(double trackWidth){
        this.trackWidth= trackWidth;
        return this;
    }

    @Deprecated
    public class ModuleFinalOutput{
        public double motorPower, servoPower;
        public void setMotorPower(double power)
        {
            this.motorPower= power;
        }
        public void setServoPower(double power){
            this.servoPower= power;
        }
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


            //  steeringServo.setAngle(targetAngle);
          //  steeringServo.update();
            this.targetAngle= currentAngle + diff;

            double alignmentFactor=  1 - Math.abs(diff)/90;
//            //alignmentFactor= Math.min(1.5 * alignmentFactor, 1);
            //speed*= alignmentFactor;
            //driveMotor.setPower(speed * alignmentFactor);
            this.targetSpeed= speed;
        }


        public double getCurrentAngle() {
            return steeringServo.getTruePosition() * 360.0;
        }


        public double getTargetAngle() {
            return steeringServo.getTargetPos() * 360.0;
        }
        public double getTargetPower(){
            return this.targetSpeed;
        }

        public double getDriveSpeed() {
            return driveMotor.getPower();
        }

        public void stop(double angle) {
            driveMotor.setPower(0);
            steeringServo.setAngle(angle);
        }

        public void write(){
            driveMotor.setPower(targetSpeed);
            steeringServo.setAngle(targetAngle);
            steeringServo.update();

        }

        public void setCoefs(PDSFCoefficients coefs) {
            steeringServo.setCoefs(coefs.p, coefs.d, coefs.s, coefs.f);
        }
    }
    public boolean ok= false;
    public PIDFController headingController;

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

                        flAngle= MathStuff.normalizeDegrees(toDegrees(flAngle)- leftFrontOffset);
                        frAngle= MathStuff.normalizeDegrees(toDegrees(frAngle)- rightFrontOffset);
                        blAngle= MathStuff.normalizeDegrees(toDegrees(blAngle)- leftBackOffset);
                        brAngle= MathStuff.normalizeDegrees(toDegrees(brAngle)- rightBackOffset);


                        fl.setState(flSpeed, flAngle);
                        fr.setState(frSpeed, frAngle);
                        bl.setState(brSpeed, blAngle);
                        br.setState(blSpeed, brAngle);

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
    public void setHeadingController(){
        headingController= new PIDFController(0.1 , Constants.strafe.getI(), 0.001, Constants.strafe.getF());

    }
    boolean joystickRot= false;


    public void stopChassisThread(){
        running= false;
    }
    volatile ElapsedTime timer;

    public void startChassisThread(){
      //  final GamepadLimiter limiter1= new GamepadLimiter(strafeX, strafeY, rotation, 6);
        timer= new ElapsedTime();
        running= true;
        Thread t= new Thread(()->{
            while (running && !Thread.currentThread().isInterrupted())
            {
                try{
                    update();
                    Thread.sleep(5);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }
        });
        t.setPriority(Thread.NORM_PRIORITY);
        t.start();

    }
    volatile public DoubleSupplier leftX, leftY, rightX;
    volatile GamepadLimiter limiter;
    public SwerveDrivetrain setSuppliers(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX){
        this.leftX= leftX;
        this.leftY= leftY;
        this.rightX= rightX;
        limiter= new GamepadLimiter(leftX, leftY, rightX, 6, 6, 20);
        return this;
    }

    public volatile double hz;
    public void update() {





            if (trackWidth == 0)
                throw new IllegalArgumentException("Track Width nesetat");
            if (wheelBase == 0)
                throw new IllegalArgumentException("Wheel Base nesetat");
            double rotation = Math.pow(rightX.getAsDouble(), 3);
            double strafeX = Math.pow(leftX.getAsDouble() , 3);
            double strafeY = Math.pow(leftY.getAsDouble(), 3);
            if (Math.abs(strafeX) > 0.02 || Math.abs(strafeY) > 0.02 || Math.abs(rotation) > 0.02) {
                radius = hypot(wheelBase, trackWidth);
                if (Math.abs(rotation) < .2 && !headingLock) {
                    headingLock = true;
                    lastAngle = robot.robotHeading;
                } else if (Math.abs(rotation) > .2) {
                    headingLock = false;
                }

                if (headingLock) {
                    double err = MathStuff.normalizeDegrees(robot.robotHeading - lastAngle);
                    headingCorrection = headingController.calculate(0, err);
                }

                rotation = headingLock ? -headingCorrection : -rotation;

                if (Math.abs(rotation) < .135)
                    rotation = 0;

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

                flAngle= MathStuff.normalizeDegrees(toDegrees(flAngle)- leftFrontOffset);
                frAngle= MathStuff.normalizeDegrees(toDegrees(frAngle)- rightFrontOffset);
                blAngle= MathStuff.normalizeDegrees(toDegrees(blAngle)- leftBackOffset);
                brAngle= MathStuff.normalizeDegrees(toDegrees(brAngle)- rightBackOffset);


                fl.setState(-flSpeed, flAngle);
                fr.setState(frSpeed, frAngle);
                bl.setState(-brSpeed, blAngle);
                br.setState(blSpeed, brAngle);

                lastFLangle = flAngle;
                lastFRangle = frAngle;
                lastBLangle = blAngle;
                lastBRangle = brAngle;
            }

            else if(Robot.shooting){
                fl.setState(0, 45);
                fr.setState(0, -45);
                bl.setState(0, -45);
                br.setState(0, 45);
            }
            else {
                fl.setState(0, toDegrees(lastFLangle));
                fr.setState(0, toDegrees(lastFRangle));
                bl.setState(0, toDegrees(lastBLangle));
                br.setState(0, toDegrees(lastBRangle));
            }
        }






    public void write(){
        if(timer== null)
            timer= new ElapsedTime();
        hz= 1/timer.seconds();
        timer.reset();
        fl.write();
        fr.write();
        bl.write();
        br.write();
    }
    public void update(double s, double f, double r) {

    }
    volatile boolean writeThreadRunning= false;
    public void startWriteThread(){
        writeThreadRunning= true;
        Thread t= new Thread(()->{
            while (writeThreadRunning && !Thread.interrupted()){
                try{
                    this.write();
                    Thread.sleep(1);
                }
                catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        t.setPriority(Thread.MAX_PRIORITY);
        t.start();
    }
    public void stopWriteThread(){
        writeThreadRunning= false;
    }

    @Override
    public void updateAuto(double strafeX, double strafeY, double rotation) {


            if (trackWidth == 0)
                throw new IllegalArgumentException("Track WIdth nesetat");
            if (wheelBase == 0)
                throw new IllegalArgumentException("Wheel Base nesetat");
//         Calculate wheel vectors
//         For rotation: each wheel moves perpendicular to its position vector
//         FL is at (-trackWidth/2, wheelBase/2), rotation adds (wheelBase/2, trackWidth/2) to velocity
//         FR is at (trackWidth/2, wheelBase/2), rotation adds (wheelBase/2, -trackWidth/2) to velocity
//         BL is at (-trackWidth/2, -wheelBase/2), rotation adds (-wheelBase/2, trackWidth/2) to velocity
//         BR is at (trackWidth/2, -wheelBase/2), rotation adds (-wheelBase/2, -trackWidth/2) to velocity
//        double smth= Math.sqrt(strafeX* strafeX + strafeY * strafeY);
//        double smth2= -.5 * smth+ .75;
            radius = hypot(trackWidth, wheelBase);
            //rotation= rotation* smth2;

            if (Math.abs(strafeX) > 0.01 || Math.abs(strafeY) > 0.01 || Math.abs(rotation) > 0.01) {
                strafeX *= -1;
                rotation *= -1.1;
                double rawMax = Math.max(Math.abs(strafeX), Math.max(Math.abs(strafeY), Math.abs(rotation)));
                rawMax = Math.abs(rawMax);
                strafeX /= rawMax;
                strafeY /= rawMax;
                rotation /= rawMax;
//            strafeX *= -1;
//            strafeY*= -1;
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
                fr.setState(-frSpeed, toDegrees(frAngle));
                bl.setState(brSpeed, toDegrees(blAngle));
                br.setState(-blSpeed, toDegrees(brAngle));

                lastFLangle = flAngle;
                lastFRangle = frAngle;
                lastBLangle = blAngle;
                lastBRangle = brAngle;
                ok = true;
            } else {
                fl.setState(0, toDegrees(lastFLangle));
                fr.setState(0, toDegrees(lastFRangle));
                bl.setState(0, toDegrees(lastBLangle));
                br.setState(0, toDegrees(lastBRangle));
                ok = false;
            }





    }

    @Override


    public SwerveDrivetrain setCoefs(PDSFCoefficients c){
        fl.setCoefs(c);
        fr.setCoefs(c);
        bl.setCoefs(c);
        br.setCoefs(c);
        return this;
    }


}


