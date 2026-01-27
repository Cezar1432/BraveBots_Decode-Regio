package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.MathStuff;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;

public class Shooter implements SubsystemInterface{
    public static BetterMotorEx m;
    public static BetterServo s;
    private final double minInput= 0, maxInput= 0, minVelOutput= 0, maxVelOutput= 0, minPosOutput= 0, maxPosOutput= 0;
    public static double p,i,d,f;
    boolean shooting= false, set= false, velocitySet= false;

    @Override
    public void update() {


    }

    public void set(){
        double vel= MathStuff.getLinearFunction(minInput, maxInput, minVelOutput, maxVelOutput, Turret.dist);
        double pos= MathStuff.getLinearFunction(minInput, maxInput, minPosOutput, maxPosOutput, minPosOutput);
        m.setVelocity(vel);
        s.setPosition(pos);

    }
    public void setBack(){
        m.setVelocity(4000);
    }
}
