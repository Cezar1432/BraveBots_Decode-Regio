package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.NonTrackingShoot;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.ResetTurret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.Shoot;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.Spit;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LimelightMath;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp(
        group = "TeleOP",name = "Telop BLUE"
)
@Deprecated
@Configurable
public class TestOpMode extends BetterOpMode {
    Robot robot;
    public SwerveDrivetrain drive;
    Thread thread2;
    boolean logicRunning= false, logicRunning2= false;
    volatile double hz2;
    volatile double  now3, last3;
    @Override
    public void initialize() {
        robot= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        robot.initialize();
        drive= new SwerveDrivetrain(robot).setWheelBase(Constants.wheelBase)
                .setTrackWidth(Constants.trackWidth)
                .setCoefs(new PDSFCoefficients(3, 0.5, 0, 0))
                .setSuppliers(() -> gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_X), () -> gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_Y), () -> gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_X));

        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS).whenPressed(Intake::toggle);
        gamepadEx1.getButton(BetterGamepad.Buttons.TOUCHPAD).whenPressed(()->{
            if (Robot.a == Alliance.BLUE) {  //Robot.pp.resetPosAndIMU();
                Robot.odo.setPosX((Turret.FIELD_LENGTH- Constants.trackWidth/100/2), DistanceUnit.INCH);
                Robot.odo.setPosY(Constants.wheelBase/2/100, DistanceUnit.INCH);
            } else {
                Robot.odo.setPosX(Constants.trackWidth/100/2, DistanceUnit.INCH);
                Robot.odo.setPosY(Constants.wheelBase/100 / 2, DistanceUnit.INCH);
            }
            double turretAngle = Robot.a == Alliance.RED ? -135 : 135;
            Turret.setAngle(turretAngle);
            Robot.odo.setHeading(90, AngleUnit.DEGREES);
        });
        gamepadEx1.getButton(BetterGamepad.Buttons.LEFT_BUMPER)
                        .whenPressed(()-> Turret.setState(Turret.State.TRACKING));
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_DOWN)
                        .whenPressed(()-> Turret.setState(Turret.State.STATIC));
        gamepadEx1.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(()->{
                    opModeScheduler.removeAllTasks();
                    opModeScheduler.addTask(new ResetTurret(Turret.State.STATIC));
                });



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
                    //noinspection BusyWait
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

        if(gamepadEx1.getButton(BetterGamepad.Buttons.RIGHT_BUMPER).wasPressed()) {
            if (Turret.getState()== Turret.State.TRACKING)
                opModeScheduler.addTask(new Shoot());
            else
                opModeScheduler.addTask(new NonTrackingShoot());
        }
        if(gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP).wasPressed())
            opModeScheduler.addTask(new ResetTurret(Turret.State.STATIC));

        if(gamepadEx1.getButton(BetterGamepad.Buttons.TRIANGLE).wasPressed())
            opModeScheduler.addTask(new Spit());

        now= System.nanoTime();

        telemetry.addData("tracking", Turret.tracking);
//        telemetry.addData("hz", 1e9/(now- last));
//        //telemetry.addData("hz2", hz);
//        telemetry.addData("hz3", hz2);
//        telemetry.addData("turret state", Turret.getState());
////        telemetry.addData("heading", robot.robotHeading);
//        telemetry.addData("x", Turret.x);
//        telemetry.addData("y", Turret.y);
//        telemetry.addData("turret angle", Turret.getAngle());
//        telemetry.addData("targetticks",Turret.targetTicks);
//        telemetry.addData("pos",Turret.getTicks()-Turret.ticksPerRevolution/2);
//        telemetry.addData("heading", LimelightMath.robotHeading);
//        telemetry.addData("shooting", Robot.shooting);#
        telemetry.addData("dist", Spindexer.colorSensor.getDistanceInCM());
        telemetry.addData("slot", Spindexer.currentSlot);
        telemetry.update();
        last= now;
        robot.update();
        drive.write();
        Turret.write();
        Shooter.write();
        //drive.setCoefs(new PDSFCoefficients(p,d,s,f));
    }
    public static double p,d,s,f;

    @Override
    public void init_start() {
        //logicRunning= true;
        logicRunning2= true;
        thread2.start();
        opModeScheduler.addTask(new ResetTurret(Turret.State.STATIC));
        Spindexer.turnBack();
        Robot.shooting= false;
    }

    @Override
    public void end() {
        logicRunning= false;
        logicRunning2= false;
    }
}
