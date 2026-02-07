package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.logic;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Task;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.ConditionalTask;

public class ShootCloseAuto implements Task {
    private final Scheduler s;
    private final double coef = 2;
    private final double increment= -0.015;
    private final double waitTime= 0.28;
    private final double waitTime2= 0.13;
    private final double vel= 1600, pos= .76;
    ElapsedTime timer;

    public ShootCloseAuto(){
        s= new Scheduler();

        //Turret.setState(Turret.State.AUTO);
        //Turret.setDegrees(-90);
                            //Shooter.setVelocity(vel);
                            Shooter.s.setPosition(.75);
        // Turret.setTracking(true);
        s.addTask(()->{
            Intake.start();
            Shooter.setVelocity(1630);
            Shooter.shooting = true;
            Spindexer.turnTo(Spindexer.Slots.SLOT_3);
        })
//                .addTask(new ConditionalTask(
//                        ()->{Turret.setDegrees(AngleUnit.normalizeDegrees(-90-AngleUnit.normalizeDegrees(Robot.odo.getHeading(AngleUnit.DEGREES))));return true;}
//                        ,
//                        ()->{Turret.setDegrees(AngleUnit.normalizeDegrees(90+(180-AngleUnit.normalizeDegrees(Robot.odo.getHeading(AngleUnit.DEGREES)))));return true;}
//
//                        ,
//                        ()->alliance == Alliance.RED
//                ))
                .addTask(Intake::reverse)
                .waitSeconds(.2)
                .addTask(Intake::start)
                .addTask(()->Math.abs(Math.abs(Shooter.motor1.getVelocity())- Math.abs(Shooter.vel))< 50 || timer.seconds()> 10)
                .waitSeconds(0.25)
                .addTask(Spindexer::shootRandom)
                .waitSeconds(waitTime)
                .addTask(()->Shooter.s.setPosition(.725))
                .waitSeconds(waitTime2)
                .addTask(()->Shooter.s.setPosition(.71))
                .waitSeconds(0.1)
                .addTask(Spindexer::turnBack)
                .addTask(()->{
                    Shooter.shooting = false;
                    Shooter.s.setPosition(.74);
                  //  Turret.setTracking(false);
                    //Shooter.setVelocity(1300);
                });
    }

    @Override
    public boolean Run() {
        if(timer== null)
            timer= new ElapsedTime();
        s.update();
        return s.done();
    }
}
