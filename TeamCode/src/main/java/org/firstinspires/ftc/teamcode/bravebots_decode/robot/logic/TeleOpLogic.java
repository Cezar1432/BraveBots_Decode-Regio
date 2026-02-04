package org.firstinspires.ftc.teamcode.bravebots_decode.robot.logic;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.ShootForta;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

public class TeleOpLogic {
    Robot robot;
    public SwerveDrivetrain drive;
    Gamepad gp1, gp2;
    BetterGamepad gp;

    public TeleOpLogic(Robot robot, Gamepad gp1, Gamepad gp2) {
        this.robot = robot;
        this.gp1 = gp1;
        this.gp2 = gp2;
        gp= new BetterGamepad(this.gp1);

        drive = new SwerveDrivetrain(robot)
                .setWheelBase(26.7)
                .setTrackWidth(34.4)
                .setCoefs(new PDSFCoefficients(3, 0.5, 0, 0))
                .setSuppliers(() -> this.gp.getDouble(BetterGamepad.Trigger.LEFT_X), () -> this.gp.getDouble(BetterGamepad.Trigger.LEFT_Y), () -> this.gp.getDouble(BetterGamepad.Trigger.RIGHT_X));


        s = new Scheduler();
        gp.getButton(BetterGamepad.Buttons.CROSS).whenPressed(Intake::toggle);
        gp.getButton(BetterGamepad.Buttons.TOUCHPAD).whenPressed(()->{
                    if (Robot.a == Alliance.BLUE) {  //Robot.pp.resetPosAndIMU();
                        Robot.odo.setPosX(-12.44 / 2, DistanceUnit.INCH);
                        Robot.odo.setPosY(16.93 / 2, DistanceUnit.INCH);
                    } else {
                        Robot.odo.setPosX(12.44 / 2, DistanceUnit.INCH);
                        Robot.odo.setPosY(16.93 / 2, DistanceUnit.INCH);
                    }
                    // Robot.pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
                    double turretAngle = Robot.a == Alliance.RED ? -135 : 135;
                    Turret.setAngle(turretAngle);
                    Robot.odo.setHeading(90, AngleUnit.DEGREES);
                });

        gp.getButton(BetterGamepad.Buttons.SQUARE).whenPressed(()->Shooter.motor1.setVelocity(-1400));
        gp.getButton(BetterGamepad.Buttons.TRIANGLE).whenPressed(()->Shooter.motor1.setVelocity(0));


    }

    public Scheduler s;
    volatile boolean logicRunning = false;

    public synchronized void updateLogic() {

//        if (gp1.crossWasPressed())
//            Intake.toggle();

//        if (gp1.circleWasPressed())
//            s.addTask(new ShootForta2());

        if (gp1.dpadLeftWasPressed())
            Spindexer.turnManuallyToLeft();
        if (gp1.dpadRightWasPressed())
            Spindexer.turnManuallyToRight();

//        startearly();
//        setPinpoint();


      //  s.update();
    }


    public volatile double logicHz, lastLogicTime= 0;
    public void startThreads() {
        // drive.startChassisThread();
        logicRunning = true;
        Shooter.setCoefs();
        Spindexer.turnBack();

        Thread thread = new Thread(() -> {
            while (logicRunning && !Thread.interrupted()) {
                try {

                    drive.update();
                    //updateLogic();
                    gp.update();
                    Turret.update();
                    Shooter.update();
                    Spindexer.update();
                    robot.update();
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        thread.setPriority(Thread.MAX_PRIORITY);
        thread.start();



    }

    public void setPinpoint() {
        if (gp1.touchpadWasPressed()) {

            //Robot.pp.setOffsets(Robot.xOffset, Robot.yOffset, DistanceUnit.INCH);

        }
    }

    public void stopThreads(){
        logicRunning= false;
        drive.stopChassisThread();
    }
    public void write(){
        drive.write();
        Turret.write();
        Shooter.write();
        logicHz= 1e9/(System.nanoTime()- lastLogicTime);
        lastLogicTime= System.nanoTime();
    }
    public synchronized void startearly(){
        if(gp1.squareWasPressed()){
//            Shooter.velocitystop =false;
//            Shooter.setRPM(Shooter.testingrpm);
        }
    }
    public void read(){
        robot.update();
    }
    private void burstingShoot(){
        if(gp1.rightBumperWasPressed()){
            Shooter.velocitystop=false;
            s.addTask(new ShootForta());
        }
    }

}
