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
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

@TeleOp
public class BetterSwerveAuto extends LinearOpMode {
    Robot robot;
    Chassis drive;
    // MultipleTelemetry t;
    Telemetry t;
    ElapsedTime time;
    SwerveDrivetrain swerveDrivetrain;
    Pose p= new Pose(24, -48, Math.toRadians(45));
    @Override
    public void runOpMode() throws InterruptedException {
        t= this.telemetry;
        robot= new Robot(hardwareMap, t, Alliance.BLUE);

        robot.initialize();
        drive= new Chassis(robot, Chassis.Control.AUTO, gamepad1,Chassis.Localizers.PINPOINT_V2, Chassis.Drivetrain.SWERVE);
        drive.drivetrain.setCoefs(new PDSFCoefficients(3,0.5,0,0));
        swerveDrivetrain= new SwerveDrivetrain(robot);
        drive.localizer.setOffsets(11.1, -5, DistanceUnit.CM);
        drive.localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        drive.setStartingPosition(new Pose());
        swerveDrivetrain.setCoefs(new PDSFCoefficients(3,0.5,0,0));
        drive.drivetrain.setTrackWidth(34.4);
        drive.drivetrain.setWheelBase(26.8);
        waitForStart();
        //drive.startChassisThread2();
        time= new ElapsedTime();
        drive.startChassisThread2();
        while (opModeIsActive()){

            if(gamepad1.crossWasPressed())
                drive.lineToLinear(p);
            if(gamepad1.circleWasPressed())
                drive.lineToConstant(p);
            if(gamepad1.triangleWasPressed())
                drive.lineToTangential(p);

            if(gamepad1.dpadUpWasPressed())
                drive.turnTo(Math.toRadians(45));
           // drive.update();

            t.addLine("cross e linear, cerc e constant si triunghi e tangential");

            t.addData("x vel", drive.localizer.getXVelocity());
            t.addData("y vel", drive.localizer.getYVelocity());
            t.addData("heading vel", drive.localizer.getHeadingVelocity());
            t.addData("XY vel", drive.localizer.getXYVelocity());
            telemetry.addData("finished", drive.finished());


            // swerveDrivetrain.drive(6,  -1.6, 0.1);
            //     t.addData("input ceva", drive.swerveDrivetrain.ok);

            // t.addData("current pose x", drive.getCurrentPosition().getX());
            // t.addData("current pose y", drive.getCurrentPosition().getY());
//            t.addData("current pose h", drive.getCurrentPosition().getTheta());
//            t.addData("target pose x", drive.targetPosition.getX());
//            //t.addData("target pose y", drive.targetPosition.getY());
//            t.addData("target pose h", drive.targetPosition.getTheta());
            //  t.addData("finshed", drive.finished());
            //    t.addData("distance from target", drive.getDistanceFromTarget());
//            t.addData("movement", drive.movementHeading);
//            t.addData("left stick x", gamepad1.left_stick_x);
//             t.addData("left stick y", gamepad1.left_stick_y);
//           // t.addData("right stick x", gamepad1.right_stick_x);
//            t.addData("x rot", drive.xRotated);
//            t.addData("y rot", drive.yRotated);
//            t.addData("theta", drive.theta);
//            t.addData("hz", 1.0/ time.seconds());
//            t.addData("target heading", drive.targetHeading);
//            t.addData("remaining distance", drive.remainingDistance);
//            t.addData("total distance", drive.totalDistance);
//            t.addData("delta heading", drive.deltaHeading);
//            t.addData("x", drive.movementStartPose.getX());
//            t.addData("mama ma sii", !Constants.useSecondaryForward);
//            t.addData("distance", Math.abs(drive.targetPosition.getX()- drive.currentPose.getX()));
//            t.addData("true sau fals", Math.abs(drive.targetPosition.getX()- drive.currentPose.getX())> Constants.forwardThreshold);


            t.addData("delta unghi", Localizer.normalizeHeading(drive.targetPosition.getTheta()- drive.targetHeading));
            t.addData("tru tru",             !Constants.useSecondaryHeading || Localizer.normalizeHeading(drive.currentPose.getTheta()- drive.targetPosition.getTheta())> Constants.headingThreshold );

            time.reset();
            t.update();
            robot.update();
            swerveDrivetrain.write();

        }
        drive.stopChassisThread();

    }
}
