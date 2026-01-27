package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

import java.util.function.BooleanSupplier;

public class TaskWithCondition {

    public Task task;
    public BooleanSupplier supplier;
    public TaskWithCondition(Task task, BooleanSupplier supplier){
        this.task= task;
        this.supplier= supplier;
    }
}
