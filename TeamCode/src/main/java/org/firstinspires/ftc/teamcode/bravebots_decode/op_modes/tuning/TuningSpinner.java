package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp
public class TuningSpinner extends BetterOpMode {


    Robot r;
    @Override
    public void initialize() {
        r= new Robot(hardwareMap, telemetry, Alliance.RED);
        r.initialize();
        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS).whenPressed(()->Spindexer.setPosition(.5));

    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void activeLoop() {
        telemetry.addData("pos 1", Spindexer.s1.getPosition());
        telemetry.addData("pos 2", Spindexer.s2.getPosition());
        telemetry.update();
        Spindexer.setPosition(Spindexer.s1.getPosition() + 0.0025 * gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_X));
    }

    @Override
    public void init_start() {

    }

    @Override
    public void end() {

    }
}
