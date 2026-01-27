package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFController;


@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(xmlTag = "BetterCRServo", name = "Better Continuous Rotation Servo")
public class BetterCRServo extends CRServoImpl implements CRServo, HardwareDevice {
    ServoEncoder encoder;
    public BetterCRServo(ServoController pdsfController, int portNumber) {
        super(pdsfController, portNumber);
    }

    public BetterCRServo(ServoController pdsfController, int portNumber, Direction direction) {
        super(pdsfController, portNumber, direction);
    }

    public enum Controller{
        PDSF, PIDF
    }
    Controller controllerType= Controller.PIDF;
    public void setControllerType(Controller controller){
        this.controllerType= controller;
    }
    public BetterCRServo(ServoController pdsfController, int portNumber, AnalogInputController a, int channel) {
        super(pdsfController, portNumber);
        encoder= new ServoEncoder(a, channel);
        encoder.setVoltages(0, 3.2);

    }

    public BetterCRServo(ServoController pdsfController, int portNumber, AnalogInputController a, int channel, double minVoltage, double maxVoltage) {
        super(pdsfController, portNumber);
        encoder= new ServoEncoder(a, channel);
        encoder.setVoltages(maxVoltage, minVoltage);
    }
    public BetterCRServo(ServoController pdsfController, int portNumber, AnalogInputController a, int channel, double minVoltage, double maxVoltage, double maxError) {
        super(pdsfController, portNumber);
        encoder= new ServoEncoder(a, channel);
        encoder.setVoltages(maxVoltage, minVoltage);
        encoder.setMaxError(maxError);
    }

    double lastSetPower= 69;
    @Override
    public void setPower(double power){
        double roundedPower = ((int)(power * 100)) / 100.0;
        if(roundedPower != lastSetPower){
            lastSetPower = roundedPower;
            int m = 1;
            if(direction == Direction.REVERSE) m = -1;
            super.setPower(power);
        }

    }

    public double power= 0, targetPos= 0;
    public enum Mode{
        PID, POWER
    }
    public Mode mode= Mode.POWER;

    public void setSpeed(double s){
        this.power= s;
        mode= Mode.POWER;

    }

    private double normalize(double pos){

        while(pos>.5) pos-=1;
        while(pos<-.5) pos+=1;
        return pos;
    }
    public void setPosition(double p){

        p= normalize(p);
        this.targetPos= p;
        mode= Mode.PID;
    }

    public void setAngle(double angle){
        this.targetPos= normalize(angle/ 360) ;
        mode= Mode.PID;

    }
    public PDSFCoefficients coefs;
    public PDSFController pdsfController = new PDSFController(0,0,0,0);
    public PIDFController pidfController = new PIDFController(0,0,0,0);
    public void setCoefs(double p, double i, double d, double f){

        if(controllerType== Controller.PDSF)
            pdsfController.setCoefficients(new PDSFCoefficients(p,i,d,f));
        else
            pidfController.setPIDF(p,i,d,f);
    }


    public double error, normalizePos;
    public void update(){
        if(mode== Mode.PID){
            normalizePos= normalize(this.getTruePosition());

            error= targetPos- normalizePos;
            if(error> .5) error-=1;
            if(error< -.5) error+=1;

            if(controllerType== Controller.PDSF)
                this.power= pdsfController.calculate(0, error);
            else
                this.power= pidfController.calculate(0, error);

            //power= controller.calculate(normalizePos, targetPos);
        }
       this.setPower(this.power);
    }

    public double normalize2(double p){
        while(p >= 1) p-=1;
        while (p< 0) p+=1;
        return p;
    }
    public double getTruePosition(){
        return this.normalize2(Math.abs(1-encoder.getTruePosition()));
    }

    public double getTheoreticalPosition(){
        return this.targetPos;
    }

    public double getMinVoltage(){
        return encoder.minVoltage;
    }
    public double getMaxVoltage(){
        return encoder.maxVoltage;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

    public double getTargetPos(){
        return targetPos;
    }








    }

