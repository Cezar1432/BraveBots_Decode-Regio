package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;

public class ResetTurretAuto implements Task {
    private final Scheduler s;
    public ResetTurretAuto(){
        s= new Scheduler();
        s.addTask(()-> Turret.m.setPower(.6))
                .waitSeconds(.3)
                .addTask(()-> Math.abs(Turret.m.getVelocity())< 20)
                .addTask(Turret::reset)
                .addTask(()-> Turret.setState(Turret.State.AUTO));
    }
    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
