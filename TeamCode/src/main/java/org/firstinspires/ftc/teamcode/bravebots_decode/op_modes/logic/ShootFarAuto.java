package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.logic;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.ConditionalTask;

public class ShootFarAuto implements Task {

    private final Scheduler s;

    private final double coef = 2;
    private final double increment= -0.015;
    private final double waitTime= 0.28;
    private final double waitTime2= 0.13;
    private final double vel= 2080, pos= .79;
    ElapsedTime timer;

    public ShootFarAuto(double time, Alliance alliance){
        s= new Scheduler();
        Spindexer.turnTo(Spindexer.Slots.SLOT_3);
        Turret.setState(Turret.State.AUTO);
        Turret.setDegrees(-110);
        //Shooter.setVelocity(vel);
        Shooter.s.setPosition(.79);
        // Turret.setTracking(true);
        s.addTask(()->{
                    Intake.start();
                    Shooter.shooting = true;
                    Spindexer.turnTo(Spindexer.Slots.SLOT_3);
        }).addTask(new ConditionalTask(
                        ()->{Turret.setDegrees(AngleUnit.normalizeDegrees(-113-AngleUnit.normalizeDegrees(Robot.odo.getHeading(AngleUnit.DEGREES))));return true;}
                ,
                        ()->{Turret.setDegrees(AngleUnit.normalizeDegrees(110+(180-AngleUnit.normalizeDegrees(Robot.odo.getHeading(AngleUnit.DEGREES)))));return true;}

                ,
                        ()->alliance == Alliance.RED
                ))
                .addTask(Intake::reverse)
                .waitSeconds(.25)
                .addTask(Intake::start)
                .addTask(()->Math.abs(Math.abs(Shooter.motor1.getVelocity())- Math.abs(Shooter.vel))< 50 || timer.seconds()> time)
                .waitSeconds(.1)
                .addTask(Spindexer::shootRandom)
                .waitSeconds(waitTime)
                .addTask(()->Shooter.s.setPosition(.775))
                .waitSeconds(waitTime2)
                .addTask(()->Shooter.s.setPosition(.76))
                .waitSeconds(0.1)
                .addTask(Spindexer::turnBack)
                .addTask(()->{
                    Shooter.shooting = false;
                    Shooter.s.setPosition(.79);
                    //  Turret.setTracking(false);
                    //Shooter.setVelocity(1300);
                });
    }

    @Override
    public boolean Run() {
        if(timer== null)
            timer= new ElapsedTime();
        s.update();
        return s.done();
    }
}
