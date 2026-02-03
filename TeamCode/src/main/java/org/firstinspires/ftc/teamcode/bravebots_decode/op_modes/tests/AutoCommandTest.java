package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

@TeleOp(group = "Tests")
public class AutoCommandTest extends LinearOpMode {
    Robot robot;
    Chassis drive;
    // MultipleTelemetry t;
    Telemetry t;
    ElapsedTime time;
    SwerveDrivetrain swerveDrivetrain;
    Pose p = new Pose(24, -48, Math.toRadians(45));

    Scheduler s;
    void initialize(){
        s.lineToConstantAsync(new Pose(20,0))
                .lineToConstantAsync(new Pose(20,20))
                .lineToConstantAsync(new Pose(0, 20))
                .lineToConstantAsync(new Pose(0,0));
    }
    boolean tru= false;
    @Override
    public void runOpMode() throws InterruptedException {
        t = this.telemetry;
        robot = new Robot(hardwareMap, t, Alliance.BLUE);

        robot.initialize();
        drive = new Chassis(robot, Chassis.Control.AUTO, gamepad1, Chassis.Localizers.PINPOINT_V2, Chassis.Drivetrain.SWERVE);
        drive.drivetrain.setCoefs(new PDSFCoefficients(3, 0.5, 0, 0));
        swerveDrivetrain = new SwerveDrivetrain(robot);
        drive.localizer.setOffsets(Constants.xOffset, Constants.yOffset, DistanceUnit.CM);
        drive.localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        drive.setStartingPosition(new Pose());
        swerveDrivetrain.setCoefs(new PDSFCoefficients(3, 0.5, 0, 0));
        drive.drivetrain.setTrackWidth(34.4);
        drive.drivetrain.setWheelBase(26.8);
        s= new Scheduler(drive);
        waitForStart();
        initialize();
        while (opModeIsActive()){
            s.update();

        }

    }
}