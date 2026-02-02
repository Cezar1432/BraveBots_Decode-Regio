package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;

@TeleOp
public class TuningSpinner extends LinearOpMode {

    Robot r;
    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        waitForStart();
        while (opModeIsActive()){
            Spindexer.setPosition(Spindexer.s1.getPosition()+ 0.0025 * gamepad1.left_stick_x);
            telemetry.addLine("Din joystick stanga gp1 ");
            telemetry.addData("pos", Spindexer.s1.getPosition());
            telemetry.update();
        }
    }
}
