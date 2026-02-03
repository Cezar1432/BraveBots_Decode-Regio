package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Poses;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

@Autonomous
public class TestAuto extends BetterOpMode {
    Robot r;
    Chassis c;
    @Override
    public void initialize() {
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        c= new Chassis(r, Chassis.Control.AUTO, Chassis.Localizers.PINPOINT_V1, Chassis.Drivetrain.SWERVE);
        c.drivetrain.setWheelBase(34.4)
                .setTrackWidth(26.7)
                .setCoefs(new PDSFCoefficients(3,.5,0,0));
        opModeScheduler.addChassis(c);
        opModeScheduler.addTask(()->c.setStartingPosition(Poses.startPose))
                .lineToConstantAsync(Poses.closeShootPose)
                .lineToConstantAsync(Poses.firstSpikes)
                .addTask(()-> c.setMaxPower(.5))
                .lineToConstantAsync(Poses.firstSpikesCollect);


    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void activeLoop() {
        telemetry.addData("current x", c.getCurrentPosition().getX());
        telemetry.addData("current y", c.getCurrentPosition().getY());
        telemetry.addData("current theta", c.getCurrentPosition().getTheta());
        telemetry.update();
        r.update();
        c.update();
    }

    @Override
    public void init_start() {

    }

    @Override
    public void end() {

    }
}
