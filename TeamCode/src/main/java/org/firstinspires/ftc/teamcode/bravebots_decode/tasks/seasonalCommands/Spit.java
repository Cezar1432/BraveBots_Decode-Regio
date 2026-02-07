package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class Spit implements Task {
    private final Scheduler s;
    public Spit(){
        s= new Scheduler()
                .addTask(Intake::reverse)
                .waitSeconds(.3)
                .addTask(Intake::stop);
    }
    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
