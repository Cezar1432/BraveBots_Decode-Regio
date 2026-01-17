package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bravebots_decode.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.useful.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.useful.Robot;

@TeleOp
@Configurable
public class SwerveOpMode extends LinearOpMode {



    public enum State{
        modul, drive, fata;
    }
    State state= State.drive;
    Robot r;
    SwerveDrivetrain swerveDriveTrain;
    PDSFCoefficients coefs;
    public static double p= 2, d, s, f;
    boolean move= false;
    void reset(){
        swerveDriveTrain.fl.steeringServo.setPower(0);
        swerveDriveTrain.fr.steeringServo.setPower(0);
        swerveDriveTrain.bl.steeringServo.setPower(0);
        swerveDriveTrain.br.steeringServo.setPower(0);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap,  Alliance.BLUE);
        r.initialize();
        swerveDriveTrain= new SwerveDrivetrain(r);
        swerveDriveTrain.setWheelBase(26.8);
        swerveDriveTrain.setTrackWidth(34.4);

        waitForStart();
        while (opModeIsActive()){

            swerveDriveTrain.setCoefs(new PDSFCoefficients(p, d, s, f));
            if(gamepad1.dpadUpWasPressed())
                state= State.drive;
            if(gamepad1.dpadDownWasPressed())
                state= State.modul;
            if(gamepad1.dpadRightWasPressed())
                state= State.fata;

            if(state== State.drive)
                swerveDriveTrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            else if(state== State.modul)
            {
                if(gamepad1.crossWasPressed())
                {
                    reset();
                    swerveDriveTrain.fl.steeringServo.setPower(1);

                }
                if(gamepad1.circleWasPressed())
                {
                    reset();
                    swerveDriveTrain.fr.steeringServo.setPower(1);

                }if(gamepad1.triangleWasPressed())
            {
                reset();
                swerveDriveTrain.bl.steeringServo.setPower(1);

            }if(gamepad1.squareWasPressed())
            {
                reset();
                swerveDriveTrain.br.steeringServo.setPower(1);

            }
            }
            else{
                swerveDriveTrain.drive(0, 0.5, 0);
            }



            telemetry.addData("state", state);
            telemetry.addData("front left  v", swerveDriveTrain.fl.steeringServo.getVoltage());
            telemetry.addData("front right v", swerveDriveTrain.fr.steeringServo.getVoltage());
            telemetry.addData("back left v", swerveDriveTrain.bl.steeringServo.getVoltage());
            telemetry.addData("back right v", swerveDriveTrain.br.steeringServo.getVoltage());
            telemetry.addLine("\n");
            telemetry.addData("front left angle", swerveDriveTrain.fl.getTargetAngle());
            telemetry.addData("front right angle", swerveDriveTrain.fr.getTargetAngle());
            telemetry.addData("back left angle", swerveDriveTrain.bl.getTargetAngle());
            telemetry.addData("back right angle", swerveDriveTrain.br.getTargetAngle());
            telemetry.addLine("\n");
            telemetry.addData("front left power", swerveDriveTrain.fl.getTargetPower());
            telemetry.addData("front tight power", swerveDriveTrain.fr.getTargetPower());
            telemetry.addData("front right angle", swerveDriveTrain.br.getCurrentAngle());
            telemetry.addData("back left power", swerveDriveTrain.bl.getTargetPower());
            telemetry.addData("back right power", swerveDriveTrain.br.getTargetPower());
            telemetry.addData("back left angle", swerveDriveTrain.bl.getCurrentAngle());
            telemetry.addData("left sticy y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
