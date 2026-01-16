package org.firstinspires.ftc.teamcode.bravebots_decode.wrappers.colorSensor;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@I2cDeviceType
@DeviceProperties(
        xmlTag = "BetterColorSensor",
        name = "Better Color Sensor"
)

public class BetterColorSensor extends RevColorSensorV3 implements HardwareDevice {
    public class ColorRangeSensorPacket {
        public double R, G, B, A;
        public double D;
        public ColorRangeSensorPacket(){
            R = G = B = 0;
            D = 0;
        }
        @Override
        public String toString(){
            return Double.toString(R) + ' ' + Double.toString(G) + ' ' + Double.toString(B);
        }
    }
    private long timeDistance = 0, timeRGB = 0;
    private double freq = 20;
    private double lowPassFilter = 1;
    public ColorRangeSensorPacket p = new ColorRangeSensorPacket();
    public ColorRangeSensorPacket RGB = new ColorRangeSensorPacket();
    public BetterColorSensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        changeLEDsettings(BroadcomColorSensor.LEDPulseModulation.LED_PULSE_100kHz, BroadcomColorSensor.LEDCurrent.CURRENT_5mA);
        timeDistance = System.currentTimeMillis();
        timeRGB = System.currentTimeMillis();
        setFreqToUpdate(20);
    }
    public void changeLEDsettings(LEDPulseModulation l, LEDCurrent c){
        setLEDParameters(l, c);
    }
    // in reads / s
    public void setFreqToUpdate(double x){
        freq =  x;
    }
    public void setLowPassFilterCoefficient(double k){
        this.lowPassFilter = k;
    }
    @Override
    public double getDistance(DistanceUnit unit){
        if(System.currentTimeMillis() - timeDistance > freq){
            p.D = unit.fromUnit(DistanceUnit.INCH, this.inFromOptical(this.rawOptical()));
            timeDistance = System.currentTimeMillis();
        }
        return p.D;
    }
    public double max,max2;
    public Colors.Balls getColorSeenBySensor(){

        return null;

    }

    public ColorRangeSensorPacket getColorPacket(){
        try {
            if (System.currentTimeMillis() - timeRGB > freq) {
                p.G = (int) (this.green() * this.lowPassFilter + p.G * (1 - lowPassFilter));
                p.R = (int) (this.red() * this.lowPassFilter + p.R * (1 - lowPassFilter));
                p.B = (int) (this.blue() * this.lowPassFilter + p.B * (1 - lowPassFilter));
                //p.A = Math.max(p.G, Math.max(p.R, p.B));

                RGB.R = p.R;
                RGB.G = p.G;
                RGB.B = p.B;

                timeRGB = System.currentTimeMillis();
            }
        } catch (Exception ignored){

        }
        return RGB;

    }
}