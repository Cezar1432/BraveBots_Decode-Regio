package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class WaitForIndexer implements Task {
    private final Scheduler s;
    public WaitForIndexer(Spindexer.Slots slot){
        s= new Scheduler();
        s.addTask(()-> {
                    Spindexer.turnTo(slot);
                    Spindexer.inSlot = true;
                })
                .waitSeconds(1)
                .addTask(()->Spindexer.inSlot= false);

    }
    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
