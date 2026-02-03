package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LimelightMath;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp
public class opmodecompletirelevant extends LinearOpMode {
    Robot r;
    long now, last= 0;
    SwerveDrivetrain drive;
    BetterGamepad gamepadEx1, gamepadEx2;
    Thread thread;
    boolean running= false;
    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        gamepadEx1 = new BetterGamepad(gamepad1);
        gamepadEx2 = new BetterGamepad(gamepad2);
        drive= new SwerveDrivetrain(r).setWheelBase(26.7)
                .setTrackWidth(34.4)
                .setCoefs(new PDSFCoefficients(3, 0.5, 0, 0))
                .setSuppliers(() -> gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_X), () -> gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_Y), () -> gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_X));

        thread= new Thread(()->{
            while (running && !Thread.currentThread().isInterrupted()){
                try{
                    drive.update();
                    Thread.sleep(2);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }
        });
        waitForStart();
        running= true;
        thread.start();
        while (opModeIsActive()){
            r.update();
            now= System.nanoTime();
            telemetry.addData("hz", 1e9/(now- last));
            telemetry.addData("x", Robot.odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("y", Robot.odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("heading", Robot.odo.getHeading(AngleUnit.DEGREES));
            last= now;
            telemetry.update();
            if(gamepad1.crossWasPressed())
                Robot.odo.update();

            drive.update();
            drive.write();

            if(gamepad1.triangleWasPressed())
                r.ll.updateRobotOrientation(LimelightMath.getLimelightUpdateAngle());

        }
        running= false;
    }
}
