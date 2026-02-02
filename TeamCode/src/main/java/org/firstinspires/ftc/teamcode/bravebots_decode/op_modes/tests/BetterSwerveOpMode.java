package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;


import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.logic.TeleOpLogic;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@TeleOp
@Configurable

public class BetterSwerveOpMode extends LinearOpMode {
    Robot robot;

    public static double p,d,ss,f;
    TeleOpLogic logic;
    long last;

    BetterServo s;
    ServoController c;
    @Override
    public void runOpMode() throws InterruptedException {

        robot= new Robot(hardwareMap, telemetry ,Alliance.BLUE);
        robot.initialize();
        logic= new TeleOpLogic(robot, gamepad1, gamepad2);

        c= hardwareMap.get(ServoController.class, "Control Hub");
        s= new BetterServo(c,2);

        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        waitForStart();




        Turret.reset();
        logic.startThreads();
        while (opModeIsActive()){



            //logic.drive.setCoefs(new PDSFCoefficients(p,d,ss,f));
            //robot.update();
            logic.write();
            long now = System.nanoTime();
            telemetry.addData("hz", 1e9/(now- last));
            telemetry.addData("logic hz", logic.logicHz);
            last= now;
//            LLResult res = LimelightMath.getResults();
//            telemetry.addData("isValid",res.isValid());
//            telemetry.addData("isValid2",robot.ll.getLatestResult().isValid());
//            if (res != null && res.isValid()) {
//                List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();
//                int i = 1;
//                for (LLResultTypes.FiducialResult fiduci : fiducials) {
//                    int apriltagID = fiduci.getFiducialId();
//                    telemetry.addData("apriltag "+i,apriltagID);
//                    i++;
//                }
//
////                telemetry.addData("LLx",res.getBotpose_MT2().getPosition().x);
////                telemetry.addData("LLy",res.getBotpose_MT2().getPosition().y);
//            }
//            telemetry.addData("x", Robot.odo.getPosX(DistanceUnit.METER));
//            telemetry.addData("y", Robot.odo.getPosY(DistanceUnit.METER));
//            telemetry.addData("heading", Robot.odo.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("xCorner", Turret.xCorner);
//            telemetry.addData("yCorner", Turret.yCorner);
//            telemetry.update();
//            telemetry.addData("ball velocity", Shooter.ballvelocity);
//            telemetry.addData("vel", Shooter.transformForMotor(Shooter.ballvelocity));
//            telemetry.addData("velocity stop", Shooter.velocitystop);
//            telemetry.addData("Color", Spindexer.colorSensor.getColorSeenBySensor());
//            telemetry.addData("distance", Spindexer.colorSensor.getDistanceInCM());
//            telemetry.addData("current slot", Spindexer.currentSlot);
//            telemetry.addData("dist ok", Spindexer.testBoolean);

           // Shooter.m.setVelocity(2000);
            telemetry.update();



        }
        logic.stopThreads();
    }
}
