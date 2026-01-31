package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.seasonalCommands;


import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;



    public class ShootForta implements Task{
        private final Scheduler s;

        public ShootForta(){
            s= new Scheduler();
            s.waitSeconds(1)
                    .addTask(()->{
                        Intake.toggle();
                        return true;
                    }).addTask(()->{
                      //  Shooter.set();
                        //TeleOpLogic.shooting = true;
                        return true;
                    })
                    .waitSeconds(0.7)
                   // .addTask(()-> Math.abs(Math.abs( Shooter.m.getRPM())- Shooter.rpm)<Shooter.rpmThreshhold)
                    .addTask(new SpinRandom());

        }

        @Override
        public boolean Run() {
            //Shooter.update();
            s.update();
            return s.done();
        }
    }



