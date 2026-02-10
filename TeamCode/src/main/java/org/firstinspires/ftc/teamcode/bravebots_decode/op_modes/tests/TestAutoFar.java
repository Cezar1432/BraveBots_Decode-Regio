package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Poses;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.logic.ShootFarAuto;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.ConditionalTask;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

@Autonomous
        (
                group = "Auto"
                ,name = "Far RED"
        )
public class TestAutoFar extends BetterOpMode {
    Robot r;
    Chassis c;
    public class HumanPlayerCycle implements Task {

        private final Scheduler s;
        public HumanPlayerCycle(Pose pose){
            s= new Scheduler(c)
                    .addTask(Intake::start)
                    .addTask(()-> c.setMaxPower(.7))
                    .lineToConstantAsync(pose, 3)
                    .lineToConstantAsync(new Pose(pose.x-10, pose.y, pose.getTheta()),1.8)
                    .lineToConstantAsync(pose, 2.5)
                    //.waitSeconds(1.6)
                    .addTask(()->c.setMaxPower(1))
                    .lineToConstantAsync(Poses.farShootPose,3)
                    .addTask(new ShootFarAuto(1.5,Alliance.RED));
        }

        @Override
        public boolean Run() {
            s.update();
            return s.done();
        }
    }
    @Override
    public void initialize() {
        r= new Robot(hardwareMap, telemetry, Alliance.RED);
        r.initialize();
        r.setOpModeType(Robot.OpModeType.AUTO);
        Robot.odo.setFreq(1);
        c= new Chassis(r, Chassis.Control.AUTO, Chassis.Localizers.PINPOINT_V1, Chassis.Drivetrain.SWERVE);
        c.drivetrain.setWheelBase(34.4)
                .setTrackWidth(26.7)
                .setCoefs(new PDSFCoefficients(3,.5,0,0));

        super.setSchedulerUpdateInInit(false);
        opModeScheduler.addChassis(c);


        c.setStartingPosition(Poses.farStartPose);
        telemetry.setMsTransmissionInterval(500);
        opModeScheduler
            .addTask(()->{
                        Turret.setState(Turret.State.AUTO);
                        Turret.setDegrees(-113);
                        Shooter.setVelocity(2500);
                        Shooter.s.setPosition(.77);
                })
        .addTask(new ShootFarAuto(3.5,Alliance.RED))
                .lineToConstantAsync(Poses.thirdSpike, 1.5)
                .lineToConstantAsync(Poses.thirdSpikesCollect,1.5)
                .lineToConstantAsync(Poses.farShootPose, 3.5)
                .addTask(new ShootFarAuto(1.5,Alliance.RED))
                .addTask(new ConditionalTask(
                        new HumanPlayerCycle(Poses.humanPlayerCollect)
                        ,
                        new LineToConstantAsync(c, Poses.farLeave)
                        ,
                        ()-> opModeTimer.seconds()< 25

                )).addTask(new ConditionalTask(
                        new HumanPlayerCycle(Poses.humanPlayer2)
                        ,
                        new LineToConstantAsync(c, Poses.farLeave)
                        ,
                        ()-> opModeTimer.seconds()< 25

                )).addTask(new ConditionalTask(
                        new HumanPlayerCycle(Poses.humanPlayer2)
                        ,
                        new LineToConstantAsync(c, Poses.farLeave)
                        ,
                        ()-> opModeTimer.seconds()< 25

                )).addTask(new ConditionalTask(
                        new HumanPlayerCycle(Poses.humanPlayer2)
                        ,
                        new LineToConstantAsync(c, Poses.farLeave)
                        ,
                        ()-> opModeTimer.seconds()< 25

                )).addTask(new ConditionalTask(
                        new HumanPlayerCycle(Poses.humanPlayer2)
                        ,
                        new LineToConstantAsync(c, Poses.farLeave)
                        ,
                        ()-> opModeTimer.seconds()< 25

                ));

        Turret.reset();


    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void activeLoop() {

        Turret.update();
        Turret.write();
        c.update();
        r.update();
        Spindexer.update();
        telemetry.addData("vel",Shooter.motor1.getVelocity());
        telemetry.update();
    }

    @Override
    public void init_start() {
        opModeTimer= new ElapsedTime();
    }

    @Override
    public void end() {

    }
}
