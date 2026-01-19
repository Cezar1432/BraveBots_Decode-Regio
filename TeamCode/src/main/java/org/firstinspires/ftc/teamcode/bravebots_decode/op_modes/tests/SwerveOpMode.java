package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.slew_rate_limiter.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.slew_rate_limiter.GamepadLimiter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;

@TeleOp
@Configurable
public class SwerveOpMode extends LinearOpMode {



    public enum State{
        modul, drive, fata;
    }

    public static double limiter= 3;

    State state= State.drive;
    Robot r;
    SwerveDrivetrain swerveDriveTrain;

    GamepadLimiter gamepadLimiter;
    SlewRateLimiter strafeLimiter, forwardLimiter, rotationLimiter;
    PDSFCoefficients coefs;

    ElapsedTime timer;
    public static double p= 3, d= 0.5, s, f;
    boolean move= false;
    void reset(){
        swerveDriveTrain.fl.steeringServo.setPower(0);
        swerveDriveTrain.fr.steeringServo.setPower(0);
        swerveDriveTrain.bl.steeringServo.setPower(0);
        swerveDriveTrain.br.steeringServo.setPower(0);
        swerveDriveTrain.fl.driveMotor.setPower(0);
        swerveDriveTrain.fr.driveMotor.setPower(0);
        swerveDriveTrain.bl.driveMotor.setPower(0);
        swerveDriveTrain.br.driveMotor.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap,  Alliance.BLUE);
        r.initialize();
        r.initialize();
        swerveDriveTrain= new SwerveDrivetrain(r);
        swerveDriveTrain.setWheelBase(26.8);
        swerveDriveTrain.setTrackWidth(34.4);

        waitForStart();
        timer= new ElapsedTime();
        forwardLimiter= new SlewRateLimiter(limiter);
        strafeLimiter= new SlewRateLimiter(limiter);
        rotationLimiter= new SlewRateLimiter(limiter);

        gamepadLimiter= new GamepadLimiter(gamepad1, limiter);

        while (opModeIsActive()){

            forwardLimiter.setLimiter(limiter);
            strafeLimiter.setLimiter(limiter);
            rotationLimiter.setLimiter(limiter);
            swerveDriveTrain.setCoefs(new PDSFCoefficients(p, d, s, f));
            if(gamepad1.dpadUpWasPressed())
                state= State.drive;
            if(gamepad1.dpadDownWasPressed())
                state= State.modul;
            if(gamepad1.dpadRightWasPressed())
                state= State.fata;

            double x,y,r;
            x= strafeLimiter.calculate(gamepad1.left_stick_x );
            y= forwardLimiter.calculate(gamepad1.left_stick_y);
            r= rotationLimiter.calculate(gamepad1.right_stick_x);
            if(state== State.drive)
                swerveDriveTrain.drive(gamepadLimiter.getLeftX(), gamepadLimiter.getLeftY(), gamepadLimiter.getRightX());
            else if(state== State.modul)
            {
                if(gamepad1.crossWasPressed())
                {
                    reset();
                    swerveDriveTrain.fl.steeringServo.setPower(1);
                    swerveDriveTrain.fl.driveMotor.setPower(1);


                }
                if(gamepad1.circleWasPressed())
                {
                    reset();
                    swerveDriveTrain.fr.steeringServo.setPower(1);
                    swerveDriveTrain.fr.driveMotor.setPower(1);


                }if(gamepad1.triangleWasPressed())
            {
                reset();
                swerveDriveTrain.bl.steeringServo.setPower(1);
                swerveDriveTrain.bl.driveMotor.setPower(1);


            }if(gamepad1.squareWasPressed())
            {
                reset();
                swerveDriveTrain.br.steeringServo.setPower(1);
                swerveDriveTrain.br.driveMotor.setPower(1);


            }
            }
            else{
                swerveDriveTrain.drive(0, 0.5, 0);
            }





            telemetry.addData("hz", 1/timer.seconds());
            timer.reset();

//            telemetry.addData("state", state);
//            telemetry.addData("front left  v", swerveDriveTrain.fl.steeringServo.getVoltage());
//            telemetry.addData("front right v", swerveDriveTrain.fr.steeringServo.getVoltage());
//            telemetry.addData("back left v", swerveDriveTrain.bl.steeringServo.getVoltage());
//            telemetry.addData("back right v", swerveDriveTrain.br.steeringServo.getVoltage());
//            telemetry.addLine("\n");
//            telemetry.addData("front left angle", swerveDriveTrain.fl.getTargetAngle());
//            telemetry.addData("front right angle", swerveDriveTrain.fr.getTargetAngle());
//            telemetry.addData("back left angle", swerveDriveTrain.bl.getTargetAngle());
//            telemetry.addData("back right angle", swerveDriveTrain.br.getTargetAngle());
//            telemetry.addLine("\n");
//            telemetry.addData("front left power", swerveDriveTrain.fl.getTargetPower());
//            telemetry.addData("front tight power", swerveDriveTrain.fr.getTargetPower());
//            telemetry.addData("front right angle", swerveDriveTrain.br.getCurrentAngle());
//            telemetry.addData("back left power", swerveDriveTrain.bl.getTargetPower());
//            telemetry.addData("back right power", swerveDriveTrain.br.getTargetPower());
//            telemetry.addData("back left angle", swerveDriveTrain.bl.getCurrentAngle());
//            telemetry.addData("left sticy y", gamepad1.left_stick_y);

            telemetry.addData("left front teoretic", swerveDriveTrain.fl.getTargetAngle());
            telemetry.addData("left front practic", swerveDriveTrain.fl.getCurrentAngle());
            telemetry.addData("right front teoretic", swerveDriveTrain.fr.getTargetAngle());
            telemetry.addData("right front practic", swerveDriveTrain.fr.getCurrentAngle());
            telemetry.addData("left back teoretic", swerveDriveTrain.bl.getTargetAngle());
            telemetry.addData("left back practic", swerveDriveTrain.bl.getCurrentAngle());
            telemetry.addData("right back teoretic", swerveDriveTrain.br.getTargetAngle());
            telemetry.addData("right back practic", swerveDriveTrain.br.getCurrentAngle());
            telemetry.update();
        }
    }
}
