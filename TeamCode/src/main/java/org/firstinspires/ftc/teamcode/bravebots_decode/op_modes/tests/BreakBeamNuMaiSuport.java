package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@TeleOp
public class BreakBeamNuMaiSuport extends LinearOpMode {
    List<LynxModule> hubs;
    DigitalChannel beamsensor;
    long loop= 0, lastLoop= 0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        hubs= hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        beamsensor  = hardwareMap.get(DigitalChannel.class, "breakBeam");
        beamsensor.setMode(DigitalChannel.Mode.INPUT);
        //breakBeam.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()){
            for(LynxModule hub:hubs)
                hub.clearBulkCache();
            telemetry.addData("status", !beamsensor.getState());
            loop= System.nanoTime();
           // telemetry.addData("hz", 1/timer.seconds());
               // telemetry.addData("hz", 1e9/(loop-lastLoop));
                telemetry.addData("transmission", telemetry.getMsTransmissionInterval());
            lastLoop= loop;
            timer.reset();
            telemetry.update();
        }
    }
}