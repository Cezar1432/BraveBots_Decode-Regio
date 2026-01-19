package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math;

public class MathStuff {
    public static double getLinearFunction(double minInput, double maxInput, double minOutput, double maxOutput, double givenInput){
        return minOutput+ (givenInput - minInput) * (maxOutput- minOutput) / (maxInput- minInput);
    }
    public static double normalizeDegrees(double degrees){
        while (degrees> 180) degrees-= 360;
        while (degrees< -180) degrees+= 360;
        return degrees;
    }

    public static double normalizeRadians(double radians){
        while (radians> Math.PI) radians-= 2* Math.PI;
        while (radians< -Math.PI) radians+= 2* Math.PI;
        return radians;
    }
}
