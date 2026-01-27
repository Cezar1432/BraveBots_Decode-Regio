package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;

public class Intake {
    public static BetterMotor motor;
    static boolean intaking = false;
    public static void start(){
        motor.setPower(-1);
        intaking= true;
    }
    public static void stop(){
        motor.setPower(0);
        intaking= false;
    }
    public static void toggle(){
        if(!intaking){
            start();
        }
        else{
            stop();
        }
    }
}
