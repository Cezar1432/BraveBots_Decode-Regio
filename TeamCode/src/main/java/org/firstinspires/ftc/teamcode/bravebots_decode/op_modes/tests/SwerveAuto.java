package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers.Localizer;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.MathStuff;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp(group = "Tests")
public class SwerveAuto extends BetterOpMode {
    Chassis drive;
    Robot r;
    double targetHeading= 0;
    @Override
    public void initialize() {
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        drive= new Chassis(r, Chassis.Control.AUTO, Chassis.Localizers.PINPOINT_V1, Chassis.Drivetrain.SWERVE);
        drive.localizer.setOffsets(11.1, -5, DistanceUnit.CM);
        drive.localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        drive.drivetrain.setCoefs(new PDSFCoefficients(3, .5,0,0))
                .setWheelBase(34.4)
                .setTrackWidth(26.7);
        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(()->drive.lineToLinear(new Pose(24, -48, Math.toRadians(targetHeading))));
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP)
                .whenPressed(()->targetHeading++);
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_DOWN)
                .whenPressed(()->targetHeading--);
        gamepadEx1.getButton(BetterGamepad.Buttons.CIRCLE)
                .whenPressed(()->drive.lineToConstant(new Pose(24, -48, Math.toRadians(targetHeading))));
        gamepadEx1.getButton(BetterGamepad.Buttons.TRIANGLE)
                .whenPressed(()->drive.lineToTangential(new Pose(24, -48, Math.toRadians(targetHeading))));

    }

    @Override
    public void initializeLoop() {

    }

    String getPoseAsString(Pose p){
        return "x: " + p.getX() + "y: " + p.getY() + "heading:" + p.getTheta();
    }
    long now, last;
    @Override
    public void activeLoop() {

        r.update();
        drive.update();
        Pose p= drive.currentPose;
        telemetry.addData("target pose", getPoseAsString(drive.targetPosition));
        telemetry.addData("current pose", getPoseAsString(p));
        now= System.nanoTime();
        telemetry.addData("hz", 1e9/(now- last));
        telemetry.update();
        last= now;



    }

    @Override
    public void init_start() {

    }

    @Override
    public void end() {

    }
}
