package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.slew_rate_limiter;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadLimiter {
    SlewRateLimiter leftX, leftY, rightX;
    Gamepad gp;
    public GamepadLimiter(Gamepad gp, double limiter){
        leftX= new SlewRateLimiter(limiter);
        leftY= new SlewRateLimiter(limiter);
        rightX= new SlewRateLimiter(limiter);
        this.gp= gp;
    }

    public double getLeftX(){
        return leftX.calculate(gp.left_stick_x);
    }
    public double getLeftY(){
        return leftY.calculate(gp.left_stick_y);
    }
    public double getRightX(){
        return rightX.calculate(gp.right_stick_x);
    }

}
