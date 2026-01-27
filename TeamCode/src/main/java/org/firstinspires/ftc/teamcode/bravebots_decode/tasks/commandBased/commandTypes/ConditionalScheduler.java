package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

import java.util.function.BooleanSupplier;

public class ConditionalScheduler implements Task {



    public enum Cardinality{
        TWO, MORE_THAN_TWO
    }
    Cardinality cardinality;
    SchedulerWithCondition[] schedulers;
    Scheduler schedulerIfTrue, schedulerIfFalse, finalScheduler;
    BooleanSupplier supplier;

    boolean checked= false;
    public ConditionalScheduler(Scheduler schedulerIfTrue, Scheduler schedulerIfFalse, BooleanSupplier supplier){
        this.schedulerIfFalse= schedulerIfFalse;
        this.schedulerIfTrue= schedulerIfTrue;
        this.supplier= supplier;
        cardinality= Cardinality.TWO;
    }

    public ConditionalScheduler(Scheduler defaultScheduler, SchedulerWithCondition... schedulers){
        this.schedulers= schedulers;
        this.finalScheduler= defaultScheduler;
        cardinality= Cardinality.MORE_THAN_TWO;
    }





    @Override
    public boolean Run() {


        if(!checked){
            if(cardinality== Cardinality.TWO)
            {
                boolean res= supplier.getAsBoolean();
                if(res)
                    finalScheduler= schedulerIfTrue;
                else
                    finalScheduler= schedulerIfFalse;
                checked= true;
            }
            else{
                checked= true;
                for(SchedulerWithCondition scheduler: schedulers){
                    boolean res= scheduler.supplier.getAsBoolean();
                    if(res){
                        this.finalScheduler= scheduler.scheduler;
                        break;
                    }
                }
            }
        }

        finalScheduler.update();
        return finalScheduler.done();
    }


}
