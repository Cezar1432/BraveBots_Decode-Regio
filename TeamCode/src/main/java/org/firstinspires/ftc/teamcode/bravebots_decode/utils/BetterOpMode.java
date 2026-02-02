package org.firstinspires.ftc.teamcode.bravebots_decode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

public abstract class BetterOpMode extends LinearOpMode {


    public BetterGamepad gamepadEx1, gamepadEx2;
    public Scheduler opModeScheduler;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1= new BetterGamepad(gamepad1);
        gamepadEx2= new BetterGamepad(gamepad2);
        opModeScheduler= new Scheduler();
        initialize();
        while (opModeInInit()){
            initializeLoop();
            opModeScheduler.update();
        }
        waitForStart();
        init_start();
        while (opModeIsActive()){
            gamepadEx1.update();
            gamepadEx2.update();
            opModeScheduler.update();
            activeLoop();
        }
        end();

    }

    public abstract void initialize();
    public abstract void initializeLoop();
    public abstract void activeLoop();
    public abstract void init_start();
    public abstract void end();
}
