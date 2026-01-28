package org.firstinspires.ftc.teamcode.bravebots_decode.robot.logic;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.ShootRandom;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;

public class TeleOpLogic {
    Robot robot;
    SwerveDrivetrain drive;
    Gamepad gp1, gp2;
    public TeleOpLogic(Robot robot, Gamepad gp1, Gamepad gp2){
        this.robot= robot;
        drive= new SwerveDrivetrain(robot);
        drive.setWheelBase(26.8);
        drive.setTrackWidth(34.4);
        drive.setCoefs(new PDSFCoefficients(3,0.5,0,0));
        this.gp1= gp1;
        this.gp2= gp2;
        drive.setSuppliers(()-> this.gp1.left_stick_x, ()-> this.gp1.left_stick_y, ()-> this.gp1.right_stick_x );
        s= new Scheduler();

    }
    public Scheduler s;
    volatile boolean logicRunning= false;
    public synchronized void updateLogic(){

        if(gp1.crossWasPressed())
            Intake.toggle();

        if(gp1.circleWasPressed())
            s.addTask(new ShootRandom());

        if(gp1.dpadLeftWasPressed())
            Spindexer.turnManuallyToLeft();
        if(gp1.dpadRightWasPressed())
            Spindexer.turnManuallyToRight();


        s.update();
    }
    public void startThreads(){
       // drive.startChassisThread();
        logicRunning= true;
        Thread thread= new Thread(()->{
            while (logicRunning && !Thread.interrupted()) {
                try {
                    drive.update();
                    updateLogic();
                    Turret.update();
                    Thread.sleep(7);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        thread.setPriority(Thread.MIN_PRIORITY);
        thread.start();


    }
    public void stopThreads(){
        logicRunning= false;
        drive.stopChassisThread();
    }
    public void write(){
        drive.write();
        Turret.write();
    }

}
