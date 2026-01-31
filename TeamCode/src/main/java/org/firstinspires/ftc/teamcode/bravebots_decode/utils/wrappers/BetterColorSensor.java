package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import static org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.Colors.*;

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
    public BetterColorSensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        setFreq(25);
        time= System.currentTimeMillis();
    }

    Color c= new Color(0,0,0);
    double d= 0;

    double freq= 20;
    double time;
    void setFreq(double freq){
        this.freq= freq;
    }
    @Override
    public double getDistance(DistanceUnit unit){
        if(System.currentTimeMillis() - time > freq){
            d = unit.fromUnit(DistanceUnit.INCH, this.inFromOptical(this.rawOptical()));
            time = System.currentTimeMillis();
        }
        return d;
    }
    public double getDistanceInCM(){
        return getDistance(DistanceUnit.CM);
    }

    public Balls getColorSeenBySensor(){
        Color output= new Color(this.red(), this.green(), this.blue());
        return Colors.getColor(output);
    }

}
