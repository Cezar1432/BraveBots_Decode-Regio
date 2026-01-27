package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.slew_rate_limiter;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class GamepadLimiter {
    SlewRateLimiter leftX, leftY, rightX;
    Gamepad gp;
    enum Mode{
        GP, SUPPLIER
    }
    Mode mode;
    public GamepadLimiter(Gamepad gp, double limiter){
        leftX= new SlewRateLimiter(limiter);
        leftY= new SlewRateLimiter(limiter);
        rightX= new SlewRateLimiter(limiter);
        this.gp= gp;
        mode= Mode.GP;
    }
    DoubleSupplier leftStickX, leftStickY, rightStickX;
    public GamepadLimiter(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, double limiter){
       leftStickX= leftX;
       leftStickY= leftY;
       rightStickX= rightX;
       this.leftX= new SlewRateLimiter(limiter);
       this.leftY= new SlewRateLimiter(limiter);
       this.rightX= new SlewRateLimiter(limiter);
       mode= Mode.SUPPLIER;

    }

    public double getLeftX(){
        if(mode== Mode.GP)
            return leftX.calculate(gp.left_stick_x);
        return leftX.calculate(leftStickX.getAsDouble());
    }
    public double getLeftY()
    {
        if(mode== Mode.GP)
            return leftY.calculate(gp.left_stick_y);
        return leftY.calculate(leftStickY.getAsDouble());
    }
    public double getRightX(){
        if(mode== Mode.GP)
            return rightX.calculate(gp.right_stick_x);
        return rightX.calculate(rightStickX.getAsDouble());
    }
    public void setLimiter(double limiter){
        rightX.setLimiter(limiter);
        leftY.setLimiter(limiter);
        leftX.setLimiter(limiter);
    }

}
