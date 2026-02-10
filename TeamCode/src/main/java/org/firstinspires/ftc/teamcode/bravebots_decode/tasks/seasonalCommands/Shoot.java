package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class Shoot implements Task {
    private final Scheduler s;
    private final double coef = 2;
    private final double increment= -0.015;
    private final double waitTime= 0.28;
    private final double waitTime2= 0.13;
    public Shoot(){
        s= new Scheduler();

        s.addTask(()-> {
                    Robot.shooting= true;
                    Shooter.shooting = true;
                    Spindexer.turnTo(Spindexer.Slots.SLOT_3);
                    Intake.start();
                    //Turret.setState(Turret.State.TRACKING);
                })
                .addTask(()->Math.abs(Math.abs(Shooter.motor1.getVelocity())- Math.abs(Shooter.vel))< Shooter.velocityThreshold)
                .addTask(Spindexer::shootRandom)
                .waitSeconds(waitTime)
                .addTask(()->Shooter.s.setPosition(Shooter.HoodPos+increment))
                .waitSeconds(waitTime2)
                .addTask(()->Shooter.s.setPosition(Shooter.HoodPos+coef* increment))
                .waitSeconds(0.5)
                .addTask(Spindexer::turnBack)
                .waitSeconds(.2)
                .addTask(Spindexer::turnBack)
                .waitSeconds(.1)
                .addTask(()->Spindexer.turnTo(Spindexer.Slots.SLOT_1))
                .addTask(()->{
                    Shooter.shooting = false;
                    Robot.shooting= false;
                    //Shooter.setVelocity(1300);
                })
                .addTask(new ResetTurret(Turret.State.STATIC));
    }

    @Override
    public boolean Run() {
        Shooter.update();
        Shooter.write();
//        Shooter.motor1.setVelocity(1700);
//        Shooter.motor2.setVelocity(1700);
//        Shooter.s.setPosition(.75);
        s.update();
        return s.done();
    }
}