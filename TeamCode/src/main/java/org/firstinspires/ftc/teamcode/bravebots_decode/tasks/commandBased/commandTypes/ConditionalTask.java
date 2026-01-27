package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

import java.util.function.BooleanSupplier;

public class ConditionalTask implements Task {

    public enum Cardinality {
        TWO, MORE_THAN_TWO
    }

    Task task, taskIfTrue, taskIfFalse;
    BooleanSupplier supplier;
    boolean checked = false;


    public Cardinality cardinality;

    public ConditionalTask(Task taskIfTrue, Task taskIfFalse, BooleanSupplier supplier) {
        this.supplier = supplier;
        this.taskIfFalse = taskIfFalse;
        this.taskIfTrue = taskIfTrue;
        cardinality = Cardinality.TWO;

    }

    TaskWithCondition[] conditionedTasks;

    public ConditionalTask(Task defaultTask, TaskWithCondition... conditionedTasks) {
        this.conditionedTasks = conditionedTasks;
        this.task = defaultTask;
        cardinality = Cardinality.MORE_THAN_TWO;
    }

    @Override
    public boolean Run() {
        if (!checked) {
            if (cardinality == Cardinality.TWO) {
                boolean res = this.supplier.getAsBoolean();
                if (res)
                    this.task = this.taskIfTrue;
                else
                    this.task = this.taskIfFalse;

                checked = true;
            }
         else
            {
            checked = true;
                for (TaskWithCondition taskWithCondition : conditionedTasks) {
            boolean res = taskWithCondition.supplier.getAsBoolean();
            if (res) {
                this.task = taskWithCondition.task;
                break;
            }
        }

    }
}
            return task.Run();
    }






    }

