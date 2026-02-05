package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class ResetTurret implements Task {

    private final Scheduler s;
    public ResetTurret(Turret.State state){
        s= new Scheduler();
        s.addTask(()->Turret.setState(Turret.State.RESET))
                .addTask(()-> Turret.m.setPower(-.5))
                .waitSeconds(.3)
                .addTask(()-> Math.abs(Turret.m.getVelocity())< 20)
                .addTask(Turret::reset)
                .addTask(()-> Turret.setState(state));
    }
    public ResetTurret(Turret.State state, double targetDegrees){
            s= new Scheduler();
            s.addTask(()-> Turret.m.setPower(-.44))
                    .waitSeconds(.3)
                    .addTask(()-> Math.abs(Turret.m.getVelocity())< 20)
                    .addTask(Turret::reset)
                    .addTask(()->
                    {
                        Turret.setState(state);
                        Turret.setDegrees(targetDegrees);
                    });
        }

    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
