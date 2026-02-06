package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Poses;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.logic.ShootCloseAuto;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.ConditionalScheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.ConditionalTask;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.ResetTurret;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@Autonomous
public class TestAuto extends BetterOpMode {
    Robot r;
    Chassis c;
    Scheduler resetTurret;
    public enum Decision{
        GATE, SPIKES;
    }
    Decision decision= Decision.SPIKES;
    public class GateCycle implements Task{

        private final Scheduler s;
        public GateCycle(){
            s= new Scheduler(c);
            s.lineToConstantAsync(Poses.gateOpenPose, 4)
                    .waitSeconds(1.5)
                    .lineToConstantAsync(Poses.closeShootPose, 2)
                    .addTask(new ShootCloseAuto());
        }

        @Override
        public boolean Run() {
            decision= Decision.GATE;
            s.update();
            return s.done();
        }
    }
    public class ShootFirstSpikes implements Task{
        private final Scheduler s;
        public ShootFirstSpikes(){
            s= new Scheduler(c);
            s.lineToConstantAsync(Poses.firstSpikesCollect, 5, .2)
                    .lineToConstantAsync(Poses.firstSpikesCollect,5, .2)
                    .lineToConstantAsync(Poses.closeShootPose,2)
                    .addTask(new ShootCloseAuto());
        }

        @Override
        public boolean Run() {
            decision= Decision.SPIKES;
            s.update();
            return s.done();
        }
    }


    Scheduler initIndexer;
    @Override
    public void initialize() {
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE);
        r.initialize();
        r.setOpModeType(Robot.OpModeType.AUTO);
        Robot.odo.setFreq(1);
        c= new Chassis(r, Chassis.Control.AUTO, Chassis.Localizers.PINPOINT_V1, Chassis.Drivetrain.SWERVE);
        c.drivetrain.setWheelBase(34.4)
                .setTrackWidth(26.7)
                .setCoefs(new PDSFCoefficients(3,.5,0,0));

        Shooter.s.setPosition(.63);
        opModeScheduler.addChassis(c);
        c.setStartingPosition(Poses.startPose);
        super.setSchedulerUpdateInInit(false);
        //c.setMaxPower(.5);
        opModeScheduler
                .addTask(()->{
                    Turret.setState(Turret.State.AUTO);
                    Turret.setDegrees(-90);
                    Shooter.setVelocity(1600);
                    Shooter.s.setPosition(.7);
                })
                .lineToConstantAsync(Poses.closeShootPose, 2.2)
                .addTask(new ShootCloseAuto())
                .lineToConstantAsync(Poses.secondSpikes, 5, Math.toRadians(15))
                .lineToConstantAsync(Poses.secondSpikesCollect, 8, Math.toRadians(30))
                .lineToConstantAsync(Poses.closeShootPose,2)
                .addTask(new ShootCloseAuto())
                .addTask(new ConditionalTask(
                        new GateCycle()
                        ,
                        new ShootFirstSpikes()
                        ,
                        ()-> opModeTimer.seconds()< 23

                ))
                .addTask(new ConditionalTask(
                        new GateCycle()
                        ,
                        new ShootFirstSpikes()
                        ,
                        ()-> opModeTimer.seconds()< 23

                )).addTask(new ConditionalTask(
                        new GateCycle()
                        ,
                        new ShootFirstSpikes()
                        ,
                        ()-> opModeTimer.seconds()< 23

                )).addTask(new ConditionalTask(
                        new GateCycle()
                        ,
                        new ShootFirstSpikes()
                        ,
                        ()-> opModeTimer.seconds()< 23

                )).addTask(new ConditionalTask(
                        new GateCycle()
                        ,
                        new ShootFirstSpikes()
                        ,
                        ()-> opModeTimer.seconds()< 23

                )).addTask(new ConditionalTask(
                        new GateCycle()
                        ,
                        new ShootFirstSpikes()
                        ,
                        ()-> opModeTimer.seconds()< 23

                ));
        resetTurret= new Scheduler();
        resetTurret.addTask(new ResetTurret(Turret.State.AUTO, 90));

        initIndexer= new Scheduler()
                .addTask(()->Spindexer.turnTo(Spindexer.Slots.SLOT_1))
                        .waitSeconds(2)
                                .addTask(()->Spindexer.turnTo(Spindexer.Slots.SLOT_2))
                                        .waitSeconds(2)
                                                .addTask(()->Spindexer.turnTo(Spindexer.Slots.SLOT_3));
        telemetry.setMsTransmissionInterval(500);


    }

    @Override
    public void initializeLoop() {

        if(gamepadEx1.getButton(BetterGamepad.Buttons.CROSS).wasPressed())
            Turret.reset();
        resetTurret.update();
        telemetry.addData("velocity", Turret.m.getVelocity());
        telemetry.addData("TurretPos",Turret.getTicks());
        telemetry.addData("TurretState",Turret.getState());
        telemetry.addData("TurretTarget",Turret.targetTicks);
        telemetry.addData("TurretDesiredDeegrees",Turret.degrees);
        telemetry.addData("current x", c.getCurrentPosition().getX());
        telemetry.addData("current y", c.getCurrentPosition().getY());
        telemetry.addData("current theta", c.getCurrentPosition().getTheta());
        telemetry.update();
        initIndexer.update();

    }

    Pose predicetedPose= new Pose();
    double last= 0, now;
    @Override
    public void activeLoop() {

        now = System.nanoTime();
        telemetry.addData("hz", 1e9/(now- last));
        telemetry.addData("decision", decision);
        last= now;
        predicetedPose= c.localizer.getPredictedPose();
       // telemetry.addData("current x", c.localizer.getActualPose().getX());
//        telemetry.addData("predicted x", predicetedPose.getX());
       // telemetry.addData("current y", c.localizer.getActualPose().getY());
//        telemetry.addData("predicted y",predicetedPose.getY());
//        telemetry.addData("current theta", c.getCurrentPosition().getTheta());
//        telemetry.addData("predicted theta", predicetedPose.getTheta());
//        telemetry.addData("distance error", c.getDistanceFromTarget());
//        telemetry.addData("heading error", c.getRadiansFromHeadingGoal());
//        telemetry.addData("finished", c.finished());
//        telemetry.addData("turret angle", Turret.getAngle());

        //telemetry.addData("x glide", c.localizer.predictedX);
        //telemetry.addData("y glide", c.localizer.predictedY);
        telemetry.addData("TurretPos",Turret.getTicks());
        telemetry.addData("TurretState",Turret.getState());
        telemetry.addData("TurretTarget",Turret.targetTicks);
        telemetry.addData("TurretDesiredDeegrees",Turret.degrees);
        telemetry.update();
        r.update();
        c.update();
        Spindexer.update();
        Turret.update();
        Turret.write();

    }

    @Override
    public void init_start() {

        opModeTimer= new ElapsedTime();
    }

    @Override
    public void end() {

    }
}
