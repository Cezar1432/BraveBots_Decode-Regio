package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.SpinRandom;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp
@Configurable
public class ShooterCalibration2 extends BetterOpMode {
    public static double velocity;
    public static double increment= 0, waitTime= 0, waitTime2= 0;
    public static class ShootFortaTunabil implements Task {
        private final Scheduler s;
        public ShootFortaTunabil(double vel){
            s= new Scheduler();

            s.addTask(()-> {
                        Shooter.motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER ,new PIDFCoefficients(.8,0,0,14.5));
                        Shooter.motor1.setVelocity(-vel);
                        Intake.start();

                    })
                    .addTask(()->Math.abs(Math.abs(Shooter.motor1.getVelocity())- Math.abs(vel))< Shooter.velocityThreshold)
                    .addTask(new SpinRandom())
                    .waitSeconds(waitTime)
                    .addTask(()->Shooter.s.setPosition(Shooter.s.getPosition()+increment))
                    .waitSeconds(waitTime2)
                    .addTask(()->Shooter.s.setPosition(Shooter.s.getPosition()+ increment))
                    .waitSeconds(2)
                    .addTask(()->Shooter.motor1.setVelocity(-1300));
        }

        @Override
        public boolean Run() {
            s.update();
            return s.done();
        }
    }
    Robot robot;
    SwerveDrivetrain drive;
    Thread thread2;
    boolean logicRunning2= false;
    @Override
    public void initialize() {
        robot= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        robot.initialize();
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP)
                .whenPressed(new ShootFortaTunabil(velocity));
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

        gamepadEx1.getButton(BetterGamepad.Buttons.SQUARE).whenPressed(()-> Shooter.motor1.setVelocity(-1400));
        gamepadEx1.getButton(BetterGamepad.Buttons.TRIANGLE).whenPressed(()->Shooter.motor1.setVelocity(0));

        thread2= new Thread(()->{
            while (logicRunning2 && !Thread.interrupted()) {
                try {

//                    now3= System.nanoTime();
//                    hz2= 1e9/(now3- last3);
//                    last3= now3;
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

    @Override
    public void activeLoop() {


        Shooter.s.setPosition(Shooter.s.getPosition()+ 0.0025*
                (gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER)- gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER)));
        telemetry.addData("pos", Shooter.s.getPosition());
        telemetry.update();

        robot.update();
        drive.write();
        Turret.write();
    }

    @Override
    public void init_start() {
        logicRunning2= true;
        thread2.start();
        Turret.reset();
    }

    @Override
    public void end() {

    }
}
