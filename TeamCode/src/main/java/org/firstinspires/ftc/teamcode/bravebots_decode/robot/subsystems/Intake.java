package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;

public class Intake {
    public static BetterMotor motor;
    public static void start(){
        motor.setPower(1);
    }
    public static void stop(){
        motor.setPower(0);
    }
}
