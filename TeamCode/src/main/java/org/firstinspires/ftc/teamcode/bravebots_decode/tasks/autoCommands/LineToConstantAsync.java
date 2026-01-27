package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands;



import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;


public class LineToConstantAsync implements Task {
    Chassis c;
    Pose p;
    boolean started;
    public LineToConstantAsync(Chassis c, Pose p){
        this.c= c;
        this.p= p;
        started= false;
    }

    @Override
    public boolean Run() {
        if(!started)
            c.lineToConstant(p);
        c.update();
        return c.finished();
    }
}
