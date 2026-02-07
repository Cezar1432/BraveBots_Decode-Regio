package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

public class LineToConstantAsyncAndTime implements Task {
    Chassis c;
    Pose p;
    boolean started;
    double time;
    public LineToConstantAsyncAndTime(Chassis c, Pose p, double time){
        this.c= c;
        this.p= p;
        this.time= time;
        started= false;
    }

    @Override
    public boolean Run() {
        if(!started) {
            c.lineToConstant(p);
            started= true;
        }
        //c.update();
        return c.finished(time);
    }
}
