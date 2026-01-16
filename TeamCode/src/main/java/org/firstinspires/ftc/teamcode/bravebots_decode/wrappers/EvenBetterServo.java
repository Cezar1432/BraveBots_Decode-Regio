package org.firstinspires.ftc.teamcode.bravebots_decode.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;



@ServoType(flavor= ServoFlavor.CUSTOM)
@DeviceProperties(xmlTag = "EvenBetterServo", name = "Even Better Servo")
public class EvenBetterServo extends ServoImpl implements Servo, HardwareDevice {


    protected double maxDegrees;
    protected ServoEncoder encoder;
    public EvenBetterServo(ServoController controller, int portNumber){
        super(controller, portNumber);
    }
    public EvenBetterServo(ServoController controller, int portNumber, double maxDegrees, AnalogInputController analogController, int analogPort, double minV, double maxV) {
        super(controller, portNumber, Direction.FORWARD);
        this.maxDegrees= maxDegrees;
        encoder= new ServoEncoder(analogController, analogPort, minV, maxV);
    }
    public EvenBetterServo(ServoController controller, int portNumber, double maxDegrees, AnalogInputController analogController, int analogPort, double minV, double maxV, double maxError) {
        super(controller, portNumber, Direction.FORWARD);
        this.maxDegrees= maxDegrees;
        encoder= new ServoEncoder(analogController, analogPort, minV, maxV, maxError);
    }
    public EvenBetterServo(ServoController controller, int portNumber, AnalogInputController analogController, int analogPort){
        super(controller, portNumber);
        this.maxDegrees= 355;
        encoder= new ServoEncoder(analogController, analogPort);

    }
    public EvenBetterServo(ServoController controller, int portNumber, Direction direction, double maxDegrees, AnalogInputController analogController, int analogPort) {
        super(controller, portNumber, direction);
        this.maxDegrees= maxDegrees;
        encoder= new ServoEncoder(analogController, analogPort);

    }
    public EvenBetterServo(ServoController controller, int portNumber, Direction direction, AnalogInputController analogController, int analogPort){
        super(controller, portNumber, direction);
        this.maxDegrees= 355;
        encoder= new ServoEncoder(analogController, analogPort);

    }
    public double angle;
    public void turnToAngle(double angle){
        setPosition(angle/maxDegrees);
        this.angle = angle;
        this.position= angle/maxDegrees;
        encoder.setTargetPosition(position);
    }
    /*  public double getAngle(){
          return controller.getServoPosition(this.getPortNumber())*maxDegrees;
      }*/
    public double getAngle(){
        return getPosition()*maxDegrees;
    }


    public void setMaxDegrees(double maxDegrees){
        this.maxDegrees= maxDegrees;
    }
    public double getMaxAngle(){
        return maxDegrees;
    }
    protected double position;

    public void turn(double pos){
        double pos1= getPosition()+ 0.0025*pos;
        setPosition(pos1);
        encoder.setTargetPosition(pos1);
    }
    public void setTargetPosition(double pos){
        this.position= pos;
        setPosition(pos);
        encoder.setTargetPosition(pos);
    }
    public boolean finished(){
        return encoder.finished();
    }
    public double getTruePosition(){
        return encoder.getTruePosition();
    }
    public void setMaxError(double err){
        encoder.setMaxError(err);
    }

    public void setEncoder(AnalogInputController controller, int portNumber){
        encoder= new ServoEncoder(controller, portNumber);
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

    public void setVoltages(double minV, double maxV){
        this.encoder.setVoltages(minV, maxV);
    }






}