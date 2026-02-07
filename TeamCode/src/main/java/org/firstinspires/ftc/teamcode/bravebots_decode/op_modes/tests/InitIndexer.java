package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@Autonomous(
        group = "Auto",
        name = "de pus preload"
)
public class InitIndexer extends BetterOpMode {


    Robot r;
    @Override
    public void initialize() {
        super.setSchedulerUpdateInInit(true);
        super.setGamepadsUpdateInInit(true);
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        opModeScheduler.addTask(()-> Spindexer.turnTo(Spindexer.Slots.SLOT_1))
                .waitSeconds(2.5)
                .addTask(()->Spindexer.turnTo(Spindexer.Slots.SLOT_2))
                .waitSeconds(2.5)
                .addTask(()->Spindexer.turnTo(Spindexer.Slots.SLOT_3));

        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(Turret::reset);
    }


    @Override
    public void initializeLoop() {

        Turret.setState(Turret.State.RESET);
        Turret.m.setPower(-.5);

    }

    @Override
    public void activeLoop() {

    }

    @Override
    public void init_start() {

    }

    @Override
    public void end() {

    }
}
