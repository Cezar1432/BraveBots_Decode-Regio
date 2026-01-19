package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@ServoType(flavor= ServoFlavor.CUSTOM)
@DeviceProperties(xmlTag = "BetterServo", name = "Better Servo")

public class BetterServoEx extends ServoImplEx implements ServoEx, HardwareDevice {


    public BetterServoEx(ServoController controller, int portNumber){
        super((ServoControllerEx)controller, portNumber, ServoConfigurationType.getStandardServoType());
    }

    public BetterServoEx(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType) {
        super(controller, portNumber, servoType);
    }



    @Deprecated
    @Override
    public void rotateByAngle(double angle, AngleUnit angleUnit) {

    }

    @Deprecated
    @Override
    public void rotateByAngle(double degrees) {

    }

    @Override
    public void turnToAngle(double angle, AngleUnit angleUnit) {

    }

    @Override
    public void turnToAngle(double angle) {

        setPosition(angle/maxDegrees);
        this.angle = angle;
    }

    @Deprecated
    @Override
    public void rotateBy(double position) {

    }

    @Deprecated

    @Override
    public void setRange(double min, double max, AngleUnit angleUnit) {

    }

    @Deprecated
    @Override
    public void setRange(double min, double max) {

    }

    @Deprecated
    @Override
    public void setInverted(boolean isInverted) {

    }

    @Deprecated
    @Override
    public boolean getInverted() {
        return false;
    }

    @Deprecated
    @Override
    public double getAngle(AngleUnit angleUnit) {
        return 0;
    }

    @Deprecated
    @Override
    public double getAngle() {
        return getPosition()*maxDegrees;

    }

    @Deprecated
    @Override
    public void disable() {

    }

    @Deprecated
    @Override
    public String getDeviceType() {
        return "";
    }


    double maxDegrees;
    public double angle;

    /*  public double getAngle(){
          return controller.getServoPosition(this.getPortNumber())*maxDegrees;
      }*/



    public void setMaxDegrees(double maxDegrees){
        this.maxDegrees= maxDegrees;
    }
    public double getMaxAngle(){
        return maxDegrees;
    }
    private double position;

    public void turn(double pos){
        setPosition(getPosition()+ 0.0025*pos);
    }

}
