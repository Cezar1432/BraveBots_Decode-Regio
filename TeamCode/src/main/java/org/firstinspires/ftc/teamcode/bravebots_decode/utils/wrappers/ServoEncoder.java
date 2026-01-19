package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;



@DeviceProperties(
        xmlTag = "ServoEncoder",
        name = "Servo Encoder"
)
@AnalogSensorType
public class ServoEncoder extends AnalogInput implements HardwareDevice {
    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public double minVoltage, maxVoltage, maxError= 0.015;
    public ServoEncoder(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    public ServoEncoder(AnalogInputController controller, int channel, double minVoltage, double maxVoltage) {
        super(controller, channel);
        this.minVoltage= minVoltage;
        this.maxVoltage= maxVoltage;
    }
    public ServoEncoder(AnalogInputController controller, int channel, double minVoltage, double maxVoltage, double maxError) {
        super(controller, channel);
        this.minVoltage= minVoltage;
        this.maxVoltage= maxVoltage;
        this.maxError= maxError;
    }
    private double percent(double number, double percent){
        return number+ percent*number/100;
    }

    public double atPosVoltage= 0;
    public boolean isStuck(){
        return this.getVoltage()> percent(atPosVoltage, 1) || this.getVoltage()< percent(atPosVoltage, -1);
    }
    public boolean atPos(){
        return this.getVoltage()>= percent(atPosVoltage,-.2) && this.getVoltage()< percent(atPosVoltage, .2);
    }
    public boolean travelling(){
        return !atPos() && !isStuck();
    }
    public void setAtPosVoltage(){
        atPosVoltage= this.getVoltage();
    }
    public enum State{
        AT_POS, TRAVEL, STUCK
    }
    public double getAtPosVoltage(){
        return atPosVoltage;
    }

    public synchronized double getTruePosition(){
        double pos= this.getVoltage()- minVoltage;
        pos= pos/(maxVoltage- minVoltage);
        pos= 1- pos;
        return pos;
    }
    double targetPos;
    public void setTargetPosition(double pos){
        this.targetPos= pos;
    }

    public boolean finished(){
        double pos= this.getTruePosition();
        return Math.abs(pos- targetPos)< maxError;

    }
    public boolean finished(double pos){
        return Math.abs(pos- targetPos)< maxError;
    }
    public void setMaxError(double err){
        this.maxError= err;
    }

    public void setVoltages(double minV, double maxV){
        this.minVoltage= minV;
        this.maxVoltage= maxV;
    }


}
