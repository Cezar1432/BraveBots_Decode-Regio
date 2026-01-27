package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;

@TeleOp
public class BetterSwerveOpMode extends LinearOpMode {
    Robot robot;
    SwerveDrivetrain drive;
    ElapsedTime time;
    Thread t;

    @Override
    public void runOpMode() throws InterruptedException {
        robot= new Robot(hardwareMap, telemetry ,Alliance.BLUE);
        robot.initialize();
        drive= new SwerveDrivetrain(robot);
        drive.setWheelBase(26.8);
        drive.setTrackWidth(34.4);
        drive.setCoefs(new PDSFCoefficients(3,0.5,0,0));
        t= Thread.currentThread();
        t.setPriority(Thread.MIN_PRIORITY);
        waitForStart();
        drive.setSuppliers(()-> gamepad1.left_stick_x, ()-> gamepad1.left_stick_y, ()-> gamepad1.right_stick_x );
       // drive.startChassisThread();
        drive.startWriteThread();
        time= new ElapsedTime();
        while (opModeIsActive()){

            robot.update();
            telemetry.addData("hz", 1/time.seconds());
            telemetry.update();
            time.reset();
            telemetry.addData("chassis hz", drive.hz);
            drive.update();
            //drive.write();
            //telemetry.update();
        }
        drive.stopWriteThread();

    }
}
