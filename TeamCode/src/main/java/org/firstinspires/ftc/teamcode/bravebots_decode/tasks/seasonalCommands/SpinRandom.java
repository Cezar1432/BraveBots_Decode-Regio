package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class SpinRandom implements Task {

    private final Scheduler s;
    public SpinRandom(){
        s= new Scheduler();
        s.addTask(()->{
            Spindexer.shootRandom();
            return true;
        })
                .waitSeconds(1)
                .addTask(()->{
                    Spindexer.turnBack();
                    return true;
                });
    }
    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
