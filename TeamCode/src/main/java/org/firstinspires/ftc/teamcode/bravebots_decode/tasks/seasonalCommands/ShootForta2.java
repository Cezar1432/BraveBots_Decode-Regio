package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class ShootForta2 implements Task {

    private final Scheduler s;
    public ShootForta2(){
        s= new Scheduler();
        s.waitSeconds(1.5)
                .addTask(new SpinRandom())
                .waitSeconds(1)
                .addTask(Spindexer::turnBack);

    }
    @Override
    public boolean Run() {
       // Shooter.setVelocityCalculated();
        Shooter.setVelocityCalculated();
        Shooter.m.setVelocity(Shooter.transformForMotor(Shooter.ballvelocity));
        s.update();
        return s.done();
    }
}
