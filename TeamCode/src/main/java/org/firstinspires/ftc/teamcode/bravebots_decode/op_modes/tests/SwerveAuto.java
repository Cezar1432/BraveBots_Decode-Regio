package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;

@TeleOp
public class SwerveAuto extends LinearOpMode {

    Robot r;
    Chassis chassis;


    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap, Alliance.BLUE);
        r.initialize();
//        chassis= new Chassis(r, Chassis.Control.AUTO ,gamepad1, Chassis.Localizers.PINPOINT_V1, Chassis.Drivetrain.SWERVE);
//        chassis.drivetrain.setCoefs(new PDSFCoefficients(3,0.5, 0,0.0));
//        chassis.setControllers();
        r.odo.resetPosAndIMU();
        waitForStart();
        while (opModeIsActive()){
            r.update();
            telemetry.addData("x", r.odo.getPosX(DistanceUnit.METER));
            telemetry.addData("y", r.odo.getPosY(DistanceUnit.METER));
            telemetry.addData("heading", r.odo.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }


    }
}
