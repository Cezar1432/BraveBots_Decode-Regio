package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;

public class SwerveAuto extends LinearOpMode {

    Robot r;
    Chassis chassis;


    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap, Alliance.BLUE);
        r.initialize();
        chassis= new Chassis(r, Chassis.Control.AUTO ,gamepad1, Chassis.Localizers.PINPOINT_V1, Chassis.Drivetrain.SWERVE);
        chassis.drivetrain.setCoefs(new PDSFCoefficients(4,0.2, 0,0.09));
        chassis.setControllers();


    }
}
