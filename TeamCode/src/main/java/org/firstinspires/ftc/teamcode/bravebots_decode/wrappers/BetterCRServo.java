package org.firstinspires.ftc.teamcode.bravebots_decode.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.bravebots_decode.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.math.PDSFController;


@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(xmlTag = "BetterCRServo", name = "Better Continuous Rotation Servo")
public class BetterCRServo extends CRServoImpl implements CRServo, HardwareDevice {
    ServoEncoder encoder;
    public BetterCRServo(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }

    public BetterCRServo(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public BetterCRServo(ServoController controller, int portNumber, AnalogInputController a, int channel) {
        super(controller, portNumber);
        encoder= new ServoEncoder(a, channel);
        encoder.setVoltages(0, 3.2);

    }

    public BetterCRServo(ServoController controller, int portNumber, AnalogInputController a, int channel, double minVoltage, double maxVoltage) {
        super(controller, portNumber);
        encoder= new ServoEncoder(a, channel);
        encoder.setVoltages(maxVoltage, minVoltage);
    }
    public BetterCRServo(ServoController controller, int portNumber, AnalogInputController a, int channel, double minVoltage, double maxVoltage, double maxError) {
        super(controller, portNumber);
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
    public PDSFController controller= new PDSFController(0,0,0,0);
    public void setCoefs(PDSFCoefficients c){

        controller.setCoefficients(c);
    }

    public double error, normalizePos;
    public void update(){
        if(mode== Mode.PID){
            normalizePos= normalize(this.getTruePosition());

            error= targetPos- normalizePos;
            if(error> .5) error-=1;
            if(error< -.5) error+=1;

            this.power= controller.calculate(0, error);

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

