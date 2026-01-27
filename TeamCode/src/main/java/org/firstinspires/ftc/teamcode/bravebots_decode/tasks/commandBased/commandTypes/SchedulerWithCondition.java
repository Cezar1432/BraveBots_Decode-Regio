package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;

import java.util.function.BooleanSupplier;

public class SchedulerWithCondition {

    public Scheduler scheduler;
    public BooleanSupplier supplier;
    public SchedulerWithCondition(Scheduler scheduler, BooleanSupplier supplier){
        this.scheduler= scheduler;
        this.supplier= supplier;
    }
}