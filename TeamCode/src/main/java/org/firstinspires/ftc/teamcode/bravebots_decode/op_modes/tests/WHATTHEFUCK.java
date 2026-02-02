package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotorEx;

import java.util.LinkedList;
import java.util.List;

@TeleOp
public class WHATTHEFUCK extends LinearOpMode {

    //    BetterMotorEx m;
//    DcMotorControllerEx controllerEx;
//    List<LynxModule> hubs= new LinkedList<>();
//    Thread t;

    Robot r;
    long now, last;
    double hz;

    @Override
    public void runOpMode() throws InterruptedException {
//        controllerEx= hardwareMap.get(DcMotorControllerEx.class, "Expansion Hub 2");
//        //m= hardwareMap.get(DcMotorEx.class, "shooter");
//        m= new BetterMotorEx(controllerEx, 3);
//        hubs= hardwareMap.getAll(LynxModule.class);
//        hubs.forEach((hub)-> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
//        t= new Thread(()->{
//            while (running) {
//                try {
//                    hubs.forEach(LynxModule::clearBulkCache);
//                    Thread.sleep(1);
//                }
//                catch (InterruptedException e){
//                    Thread.currentThread().interrupt();
//                }
//            }
//        });
//
//        waitForStart();
//        running= true;
//        t.start();
//        while (opModeIsActive()){
//
//           // hubs.forEach(LynxModule::clearBulkCache);
//            telemetry.addData("vel", m.getVelocity());
//            telemetry.addData("ticks", m.getCurrentPosition());
//            telemetry.update();
//        }
//
//    }
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        telemetry.setMsTransmissionInterval(1000);

        waitForStart();
        while (opModeIsActive()){
            now= System.nanoTime();
            telemetry.addData("hz", 1e9/(now- last));
            telemetry.update();
            r.update();
            last= now;
        }
    }
}
