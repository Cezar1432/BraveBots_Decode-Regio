package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp(
        name = "LinearOpModeEx Test"
)
public class TestOpMode extends BetterOpMode {
    Robot robot;
    public SwerveDrivetrain drive;
    Thread thread;
    Thread thread2;
    boolean logicRunning= false, logicRunning2= false;
    boolean allianceSet= false;
    volatile double hz, hz2;
    volatile double now2, last2, now3, last3;
    @Override
    public void initialize() {
        robot= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        robot.initialize();
        drive= new SwerveDrivetrain(robot).setWheelBase(26.7)
                .setTrackWidth(34.4)
                .setCoefs(new PDSFCoefficients(3, 0.5, 0, 0))
                .setSuppliers(() -> gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_X), () -> gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_Y), () -> gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_X));

        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS).whenPressed(Intake::toggle);
        gamepadEx1.getButton(BetterGamepad.Buttons.TOUCHPAD).whenPressed(()->{
            if (Robot.a == Alliance.BLUE) {  //Robot.pp.resetPosAndIMU();
                Robot.odo.setPosX(-12.44 / 2, DistanceUnit.INCH);
                Robot.odo.setPosY(16.93 / 2, DistanceUnit.INCH);
            } else {
                Robot.odo.setPosX(12.44 / 2, DistanceUnit.INCH);
                Robot.odo.setPosY(16.93 / 2, DistanceUnit.INCH);
            }
            double turretAngle = Robot.a == Alliance.RED ? -135 : 135;
            Turret.setAngle(turretAngle);
            Robot.odo.setHeading(90, AngleUnit.DEGREES);
        });
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP)
                        .whenPressed(()-> Turret.setTracking(true));
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_DOWN)
                        .whenPressed(()-> Turret.setTracking(false));

        gamepadEx1.getButton(BetterGamepad.Buttons.SQUARE).whenPressed(()-> Shooter.motor1.setVelocity(-1400));
        gamepadEx1.getButton(BetterGamepad.Buttons.TRIANGLE).whenPressed(()->Shooter.motor1.setVelocity(0));

        thread2= new Thread(()->{
            while (logicRunning2 && !Thread.interrupted()) {
                try {

                    now3= System.nanoTime();
                    hz2= 1e9/(now3- last3);
                    last3= now3;
                    drive.update();
                    //updateLogic();
                    Turret.update();
                    Shooter.update();
                    Spindexer.update();
                    //robot.update();
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

        });
        thread2.setPriority(5);

        telemetry.setMsTransmissionInterval(1000);


    }

    @Override
    public void initializeLoop() {

    }

    long now, last= 0;
    @Override
    public void activeLoop() {

        now= System.nanoTime();
        telemetry.addData("hz", 1e9/(now- last));
        //telemetry.addData("hz2", hz);
        telemetry.addData("hz3", hz2);
        telemetry.addData("tracking", Turret.tracking);
        telemetry.addData("heaing", robot.robotHeading);
        telemetry.addData("x", Turret.x);
        telemetry.addData("y", Turret.y);
        telemetry.addData("xCorner", Turret.xCorner);
        telemetry.addData("yCorner", Turret.yCorner);
        telemetry.update();
        last= now;
        robot.update();
        drive.write();
        Turret.write();
    }

    @Override
    public void init_start() {
        //logicRunning= true;
        logicRunning2= true;
        thread2.start();
        Turret.reset();
        Spindexer.turnBack();
    }

    @Override
    public void end() {
        logicRunning= false;
        logicRunning2= false;
    }
}
