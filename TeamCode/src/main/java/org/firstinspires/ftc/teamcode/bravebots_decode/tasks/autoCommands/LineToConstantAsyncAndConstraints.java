package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

public class LineToConstantAsyncAndConstraints implements Task {
    Chassis c;
    Pose p;
    boolean started;
    double dist, rad;
    public LineToConstantAsyncAndConstraints(Chassis c, Pose p, double dist, double rad){
        this.c= c;
        this.p= p;
        this.dist= dist;
        this.rad= rad;
        started= false;
    }

    @Override
    public boolean Run() {
        if(!started) {
            c.lineToConstant(p);
            started= true;
        }
        //c.update();
        return c.finished(dist, rad);
    }
}
