package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.logic;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class ShootCloseAuto implements Task {
    private final Scheduler s;
    private final double coef = 2;
    private final double increment= -0.015;
    private final double waitTime= 0.28;
    private final double waitTime2= 0.13;
    private final double vel= 1800, pos= .7578;
    ElapsedTime timer;

    public ShootCloseAuto(){
        s= new Scheduler();

        Turret.setState(Turret.State.AUTO);
        Turret.setDegrees(-90);
        //                    Shooter.setVelocity(vel);
        //                    Shooter.s.setPosition(pos);
        // Turret.setTracking(true);
        s.addTask(Intake::start)
                .addTask(()->Math.abs(Math.abs(Shooter.motor1.getVelocity())- Math.abs(Shooter.vel))< Shooter.velocityThreshold || timer.seconds()> 2)
                .addTask(Spindexer::shootRandom)
                .waitSeconds(waitTime)
                .addTask(()->Shooter.s.setPosition(Shooter.HoodPos+increment))
                .waitSeconds(waitTime2)
                .addTask(()->Shooter.s.setPosition(Shooter.HoodPos+coef* increment))
                .waitSeconds(0.5)
                .addTask(Spindexer::turnBack)
                .addTask(()->{
                    Shooter.shooting = false;
                    Turret.setTracking(false);
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
