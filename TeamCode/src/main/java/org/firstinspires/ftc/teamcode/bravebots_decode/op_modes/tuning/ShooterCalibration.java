package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.logic.TeleOpLogic;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.SpinRandom;

@TeleOp
@Configurable
public class ShooterCalibration extends LinearOpMode {
    public static double velocity;
    public class ShootFortaTunabil implements Task {
        private final Scheduler s;
        public ShootFortaTunabil(double vel){
            s= new Scheduler();

            s.addTask(()-> {
                                Shooter.m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER ,new PIDFCoefficients(.8,0,0,14.5));
                                Shooter.m.setVelocity(-vel);
                        Intake.start();

                            })
                    .addTask(()->Math.abs(Math.abs(Shooter.m.getVelocity())- Math.abs(vel))< Shooter.velocityThreshold)
                    .addTask(new SpinRandom())
                    .waitSeconds(2)
                    .addTask(()->Shooter.m.setVelocity(-600));
        }

        @Override
        public boolean Run() {
            s.update();
            return s.done();
        }
    }
    Robot r;
    Scheduler s;
    TeleOpLogic logic;
    double lastTime;
    @Override
    public void runOpMode() throws InterruptedException {
        r= new Robot(hardwareMap, telemetry, Alliance.RED);
        r.initialize();
        s= new Scheduler();
        logic= new TeleOpLogic(r, gamepad1, gamepad2);
        waitForStart();
        logic.startThreads();
        Turret.reset();
        while (opModeIsActive()){
            Shooter.s.setPosition(Shooter.s.getPosition()+ 0.0025 * (gamepad1.right_trigger- gamepad1.left_trigger));
            if(gamepad1.dpadUpWasPressed())
                s.addTask(new ShootFortaTunabil(velocity));

            logic.write();
            //logic.read();
            s.update();
            telemetry.addData("dist", Turret.dist);
            telemetry.addData("actual velocity", Shooter.m.getVelocity());
            telemetry.addData("velocity", velocity);
            telemetry.addData("hood pos", Shooter.s.getPosition());

            telemetry.addData("hz", 1000/(lastTime- System.currentTimeMillis()));
            telemetry.addData("pos", Spindexer.s1.getPosition());
            lastTime= System.currentTimeMillis();
            telemetry.update();


        }
        logic.stopThreads();

    }
}
