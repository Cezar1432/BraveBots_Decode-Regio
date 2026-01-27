package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.logic.TeleOpLogic;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
public class BetterSwerveOpMode extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(BetterSwerveOpMode.class);
    Robot robot;
    SwerveDrivetrain drive;
    ElapsedTime time;
    Thread t;
    TeleOpLogic logic;
    long last;

    BetterServo s;
    ServoController c;
    @Override
    public void runOpMode() throws InterruptedException {
        robot= new Robot(hardwareMap, telemetry ,Alliance.BLUE);
        robot.initialize();
        logic= new TeleOpLogic(robot, gamepad1, gamepad2);

        c= hardwareMap.get(ServoController.class, "Control Hub");
        s= new BetterServo(c,2);

        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        waitForStart();

        logic.startThreads();
        while (opModeIsActive()){



            if(gamepad1.triangleWasPressed())
                s.setPosition(1);
            if(gamepad1.squareWasPressed())
                s.setPosition(0);

            robot.update();
            logic.write();
            long now = System.nanoTime();
            telemetry.addData("hz", 1e9/(now- last));
            telemetry.update();
            last= now;


        }
        logic.stopThreads();
    }
}
