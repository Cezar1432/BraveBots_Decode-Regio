package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;


@ServoType(flavor= ServoFlavor.CUSTOM)
@DeviceProperties(xmlTag = "BetterServo", name = "Better Servo")
public class BetterServo extends ServoImpl implements Servo, HardwareDevice {


    double maxDegrees;
    public BetterServo(ServoController controller, int portNumber, double maxDegrees) {
        super(controller, portNumber, Direction.FORWARD);
        this.maxDegrees= maxDegrees;
    }
    public BetterServo(ServoController controller, int portNumber){
        super(controller, portNumber);
        this.maxDegrees= 355;
    }
    public BetterServo(ServoController controller, int portNumber, Direction direction, double maxDegrees) {
        super(controller, portNumber, direction);
        this.maxDegrees= maxDegrees;
    }
    public BetterServo(ServoController controller, int portNumber, Direction direction){
        super(controller, portNumber, direction);
        this.maxDegrees= 355;
    }
    public double angle;
    public void turnToAngle(double angle){
        setPosition(angle/maxDegrees);
        this.angle = angle;
    }
    /*  public double getAngle(){
          return controller.getServoPosition(this.getPortNumber())*maxDegrees;
      }*/
    public double getAngle(){
        return getPosition()*maxDegrees;
    }


    public BetterServo setMaxDegrees(double maxDegrees){
        this.maxDegrees= maxDegrees;
        return this;
    }
    public double getMaxAngle(){
        return maxDegrees;
    }
    private double position;

    public void turn(double angle){
        setPosition(getPosition()+ angle/maxDegrees);
    }







}
