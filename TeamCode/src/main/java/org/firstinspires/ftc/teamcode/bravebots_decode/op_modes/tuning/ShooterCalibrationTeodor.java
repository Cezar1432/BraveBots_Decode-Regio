package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;
@TeleOp
@Configurable
public class ShooterCalibrationTeodor extends LinearOpMode {
    Robot r;
    SwerveDrivetrain drive;
    Thread thread2;
    Scheduler currentTask;
    public static double velocity;
    public static double increment= 0, waitTime= 0, waitTime2= 0;
    public static double p=0,d=0,i=0,f=0;
    public BetterGamepad gamepadEx1, gamepadEx2;
    boolean logicRunning2= false;
    public static class ShootFortaTunabil implements Task {
        private final Scheduler s;
        public ShootFortaTunabil(double vel){
            s= new Scheduler();

            s.addTask(()-> {
                        Shooter.setVelocity(velocity);
                        Intake.start();
            })
                    .addTask(()->Math.abs(Math.abs(Shooter.motor1.getVelocity())- Math.abs(vel))< Shooter.velocityThreshold)
                    .addTask(Spindexer::shootRandom)
                    .waitSeconds(waitTime)
                    .addTask(()->Shooter.s.setPosition(Shooter.s.getPosition()+increment))
                    .waitSeconds(waitTime2)
                    .addTask(()->Shooter.s.setPosition(Shooter.s.getPosition()+ increment))
                    .waitSeconds(1)
                    .addTask(Spindexer::turnBack)
                    .addTask(()->{
                        Shooter.setVelocity(1300);
                    });
        }

        @Override
        public boolean Run() {
            s.update();
            return s.done();
        }
    }

    double now, last=0;
    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
//        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP)
//                .whenPressed(()->
//                        new ShootFortaTunabil(()->velocity));
        gamepadEx1 = new BetterGamepad(gamepad1);
        gamepadEx2 = new BetterGamepad(gamepad2);
        drive= new SwerveDrivetrain(r).setWheelBase(26.7)
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
        currentTask = new Scheduler();
//        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP).whenPressed(
//                ()->{
//                    nigg= true;
//                    opModeScheduler.addTask(new ShootFortaTunabil(()->velocity));
//                }
//6        );
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


        waitForStart();
        Turret.reset();
        Spindexer.turnBack();
        logicRunning2 = true;
        thread2.start();
        Turret.setTracking(true);
        while (opModeIsActive()){
            if(gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_DOWN).wasPressed()){
                Shooter.motor1.setVelocity(-velocity);
                Shooter.motor2.setVelocity(-velocity);
            }
            if(gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP).wasPressed()) {
                currentTask.addTask(new ShootFortaTunabil(velocity));
                //nigg= true;
            }
            Shooter.s.setPosition(Shooter.s.getPosition()+ 0.0025*
                    (gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER)- gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER)));
            now= System.nanoTime();
            telemetry.addData("hz", 1e9/(now- last));
            last= now;
            telemetry.addData("pos", Shooter.s.getPosition());
            telemetry.addData("vel", Shooter.motor1.getVelocity());
            telemetry.addData("vel 2", Shooter.motor2.getVelocity());
            telemetry.addData("encoderTurret",Turret.m.getCurrentPosition());
            telemetry.addData("p", Shooter.motor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
            telemetry.addData("d", Shooter.motor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d);
            telemetry.addData("i", Shooter.motor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i);
            telemetry.addData("f", Shooter.motor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
            telemetry.addData("tracking", Turret.tracking);
            telemetry.update();
            gamepadEx1.update();

            r.update();
            currentTask.update();
            drive.write();
            Turret.write();
        }
        logicRunning2 = false;
    }
}
