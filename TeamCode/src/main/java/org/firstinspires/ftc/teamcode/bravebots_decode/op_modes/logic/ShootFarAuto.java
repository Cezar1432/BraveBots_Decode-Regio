package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.logic;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class ShootFarAuto implements Task {

    private final Scheduler s;

    private final double coef = 2;
    private final double increment= -0.015;
    private final double waitTime= 0.28;
    private final double waitTime2= 0.13;
    private final double vel= 2080, pos= .79;
    ElapsedTime timer;

    public ShootFarAuto(double time){
        s= new Scheduler();

        Turret.setState(Turret.State.AUTO);
        Turret.setDegrees(-110);
        //Shooter.setVelocity(vel);
        Shooter.s.setPosition(.79);
        // Turret.setTracking(true);
        s.addTask(()->{
                    Intake.start();Shooter.shooting = true;})
                .addTask(()->Math.abs(Math.abs(Shooter.motor1.getVelocity())- Math.abs(Shooter.vel))< 100 || timer.seconds()> time)
                .addTask(Spindexer::shootRandom)
                .waitSeconds(waitTime)
                .addTask(()->Shooter.s.setPosition(.775))
                .waitSeconds(waitTime2)
                .addTask(()->Shooter.s.setPosition(.76))
                .waitSeconds(0.5)
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
