package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes;


import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class Wait implements Task {
    double wait;
    double startTime;
    public Wait(double sec){
        wait=(int)(sec*1000);
        startTime= -1;
    }

    @Override
    public boolean Run() {
        if(startTime== -1)
            startTime= System.currentTimeMillis();
        boolean done = System.currentTimeMillis()- startTime>= wait;
        if(done)
            startTime= -1;
        return done;
    }
}
