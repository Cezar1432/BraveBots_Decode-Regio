package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Objects;



@DeviceProperties(
        xmlTag = "betterMotor",
        name = "Better Motor",
        description = "generic motor",
        builtIn = false
)
@MotorType(
        ticksPerRev = 1550,
        gearing = 55,
        maxRPM = 6000,
        achieveableMaxRPMFraction = 1.0,
        orientation = Rotation.CCW
)

public class BetterMotor extends DcMotorImplEx implements DcMotorEx, HardwareDevice {
    private double lastSetPower = 69;
    private static double MAX_VELOCITY;
    private Direction direction;

    double cachingTolerance;
    public BetterMotor(DcMotorController controller, int portNumber) {

        super(controller, portNumber);
        cachingTolerance= 0;
    }
    public BetterMotor(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction, ZeroPowerBehavior zeroPowerBehavior){
        super(controller, portNumber, direction);
        this.direction = direction;
        this.setZeroPowerBehavior(zeroPowerBehavior);
        cachingTolerance= 0;

    }
    public BetterMotor(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction){
        super(controller, portNumber, direction);
        this.direction = direction;
        this.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        cachingTolerance= 0;

    }
    public BetterMotor(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction, @NonNull MotorConfigurationType mct){
        super(controller, portNumber, direction, Objects.requireNonNull(mct));
        this.direction = direction;

        mct.setAchieveableMaxRPMFraction(1.0);
        this.controller.setMotorType(portNumber, mct.clone());
        cachingTolerance= 0;

    }
    public BetterMotor(DcMotor motor){
        super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
        motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        lastSetPower = 69;
        cachingTolerance= 0;

    }
    /**
     * MaxVelocity is measured in [outputDiameter]_(SI) / s, we recommend using m/s
     * <p>
     * @param gearRatio is measured as output / input
     * </p>
     * */
    public void setMaxVelocity(double MaxRPM, double gearRatio, double CPR){
        MAX_VELOCITY = MaxRPM * gearRatio * CPR;
    }
    public double getMaxVelocity(){
        return MAX_VELOCITY;
    }
    @Override
    public void setPower(double power){
        if(Math.abs(power- lastSetPower)> cachingTolerance) {
            super.setPower(power);
            lastSetPower= power;
        }
    }

    public BetterMotor setCachingTolerance(double tolerance){
        this.cachingTolerance= tolerance;

        return this;
    }
    public void setAproxVelocity(double velocity){
        setPower(velocity / MAX_VELOCITY);
    }
    public boolean isStill(){
        return Math.abs(this.getVelocity())<1e-2;
    }
    public void resetEncoder(){
        controller.setMotorMode(this.getPortNumber(), RunMode.STOP_AND_RESET_ENCODER);
        controller.setMotorMode(this.getPortNumber(), RunMode.RUN_WITHOUT_ENCODER);
    }

}
