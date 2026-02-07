package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Poses;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.logic.ShootCloseAuto;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.ConditionalTask;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands.ResetTurret;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@Autonomous(
        group = "Auto",name="Close BLUE"
)
public class TestAutoBlue extends BetterOpMode {
    Robot r;
    Chassis c;
    Scheduler resetTurret;
    public enum Decision{
        GATE, SPIKES;
    }
    TestAuto.Decision decision= TestAuto.Decision.SPIKES;
    public class GateCycle implements Task{

        private final Scheduler s;
        public GateCycle(){
            s= new Scheduler(c);

            s.lineToConstantAsync(Poses.intermediateGateOpen.mirrorOnYAxis(), 10, Math.toRadians(30))
                    .lineToConstantAsync(Poses.gateOpenPose.mirrorOnYAxis(), 1.5)
                    .waitSeconds(1.5)
                    .lineToConstantAsync(Poses.intermediateGateOpen.mirrorOnYAxis(), .6)
                    .lineToConstantAsync(Poses.closeShootPose.mirrorOnYAxis(), 2.5)
                    .addTask(new ShootCloseAuto());
        }

        @Override
        public boolean Run() {
            decision= TestAuto.Decision.GATE;
            s.update();
            return s.done();
        }
    }
    public class ShootFirstSpikes implements Task{
        private final Scheduler s;
        public ShootFirstSpikes(){
            s= new Scheduler(c);
            s
                    .lineToConstantAsync(Poses.firstSpikesCollect.mirrorOnYAxis(),3)
                    .lineToConstantAsync(Poses.closeShootPose.mirrorOnYAxis(),2.5)
                    .addTask(new ShootCloseAuto());
        }

        @Override
        public boolean Run() {
            decision= TestAuto.Decision.SPIKES;
            s.update();
            return s.done();
        }
    }

    boolean running = false;
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

        Shooter.s.setPosition(.74);
        opModeScheduler.addChassis(c);
        c.setStartingPosition(Poses.startPose.mirrorOnYAxis());
        super.setSchedulerUpdateInInit(false);
        //c.setMaxPower(.5);
        opModeScheduler
                .addTask(()->{
                    Turret.setState(Turret.State.AUTO);
                    Turret.setDegrees(90);
                    Shooter.setVelocity(2000);
                    Shooter.s.setPosition(.74);
                })
                .lineToConstantAsync(Poses.closeShootPose.mirrorOnYAxis(), 2.5)
                .addTask(()->{
                    //Shooter.setVelocity(1630);
                })
                .addTask(new ShootCloseAuto())
                .addTask(()->{
                    Turret.setState(Turret.State.AUTO);
                    Turret.setDegrees(90);
                    Shooter.setVelocity(1800);
                    Shooter.s.setPosition(.74);
                })
                .lineToConstantAsync(Poses.secondSpikes.mirrorOnYAxis(), 5, Math.toRadians(15))
                .lineToConstantAsync(Poses.secondSpikesCollect.mirrorOnYAxis(), 8, Math.toRadians(30))
                .lineToConstantAsync(Poses.secondSpikes.mirrorOnYAxis(), .8)
                .lineToConstantAsync(Poses.closeShootPose.mirrorOnYAxis(),3)
                .addTask(new ShootCloseAuto())
                .addTask(()->{
                    Turret.setState(Turret.State.AUTO);
                    Turret.setDegrees(90);
                    Shooter.setVelocity(1800);
                    Shooter.s.setPosition(.74);
                })
                /*.addTask(new ConditionalTask(
                        new GateCycle()
                        ,
                        new ShootFirstSpikes()
                        ,
                        ()-> opModeTimer.seconds()< 23

                ))*/
                .addTask(new GateCycle())
                .addTask(()->{
                    Turret.setState(Turret.State.AUTO);
                    Turret.setDegrees(90);
                    Shooter.setVelocity(1800);
                    Shooter.s.setPosition(.74);
                })
                .addTask(new ShootFirstSpikes())
                .addTask(()->{
                    Turret.setState(Turret.State.AUTO);
                    Turret.setDegrees(90);
                    Shooter.setVelocity(2000);
                    Shooter.s.setPosition(.74);
                })
                .lineToConstantAsync(Poses.firstSpikesCollect.mirrorOnYAxis(),2);
//                .addTask(new GateCycle())
//                .addTask(new GateCycle())
//                .addTask(new ConditionalTask(
//                        new GateCycle()
//                        ,
//                        new ShootFirstSpikes()
//                        ,
//                        ()-> opModeTimer.seconds()< 23
//
//                )).addTask(new ConditionalTask(
//                        new GateCycle()
//                        ,
//                        new ShootFirstSpikes()
//                        ,
//                        ()-> opModeTimer.seconds()< 23
//
//                )).addTask(new ConditionalTask(
//                        new GateCycle()
//                        ,
//                        new ShootFirstSpikes()
//                        ,
//                        ()-> opModeTimer.seconds()< 23
//
//                ));
        resetTurret= new Scheduler();
        resetTurret.addTask(new ResetTurret(Turret.State.AUTO));

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
        r.update();
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
        //initIndexer.update();

    }

    Pose predicetedPose= new Pose();
    double last= 0, now;
    @Override
    public void activeLoop() {

        now = System.nanoTime();
        telemetry.addData("hz", 1e9/(now- last));
        telemetry.addData("decision", decision);
        telemetry.addData("hood pos", Shooter.s.getPosition());
        last= now;
//       // predicetedPose= c.localizer.getPredictedPose();
//        // telemetry.addData("current x", c.localizer.getActualPose().getX());
////        telemetry.addData("predicted x", predicetedPose.getX());
//        // telemetry.addData("current y", c.localizer.getActualPose().getY());
////        telemetry.addData("predicted y",predicetedPose.getY());
////        telemetry.addData("current theta", c.getCurrentPosition().getTheta());
////        telemetry.addData("predicted theta", predicetedPose.getTheta());
////        telemetry.addData("distance error", c.getDistanceFromTarget());
////        telemetry.addData("heading error", c.getRadiansFromHeadingGoal());
////        telemetry.addData("finished", c.finished());
////        telemetry.addData("turret angle", Turret.getAngle());
//
//        //telemetry.addData("x glide", c.localizer.predictedX);
//        //telemetry.addData("y glide", c.localizer.predictedY);
//        telemetry.addData("TurretVel",Turret.m.getVelocity());
//        telemetry.addData("velocity1",Shooter.motor1.getVelocity());
//        telemetry.addData("velocity2",Shooter.motor2.getVelocity());
//        telemetry.addData("vel",Shooter.vel);
//        //telemetry.addData("TurretPos",Turret.getTicks());
//        //telemetry.addData("TurretState",Turret.getState());
//        //telemetry.addData("TurretTarget",Turret.targetTicks);
//        //telemetry.addData("TurretDesiredDeegrees",Turret.degrees);
//        telemetry.update();
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
