package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers.Localizer;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp
public class BetterSwerveAuto extends BetterOpMode {

    Chassis drive;
    Robot r;
    @Override
    public void initialize() {
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        drive= new Chassis(r, Chassis.Control.AUTO, Chassis.Localizers.PINPOINT_V1, Chassis.Drivetrain.SWERVE);
        drive.localizer.setOffsets(Constants.xOffset, Constants.yOffset, DistanceUnit.CM);
        drive.localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        drive.drivetrain.setCoefs(new PDSFCoefficients(3, .5,0,0))
                .setWheelBase(34.4)
                .setTrackWidth(26.7);

        opModeScheduler.addChassis(drive);
        drive.setStartingPosition(new Pose(0,0, Math.toRadians(-90)));

    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void activeLoop() {
        r.update();
        drive.update();
        if(gamepadEx1.getButton(BetterGamepad.Buttons.CROSS).wasPressed())
            opModeScheduler.lineToConstantAsync(new Pose(24, -48,Math.toRadians(-90)));

        telemetry.addData("current x", drive.getCurrentPosition().getX());
        telemetry.addData("current y", drive.getCurrentPosition().getY());
        telemetry.addData("current theta", drive.getCurrentPosition().getTheta());

        telemetry.update();
    }

    @Override
    public void init_start() {

    }

    @Override
    public void end() {

    }
}
