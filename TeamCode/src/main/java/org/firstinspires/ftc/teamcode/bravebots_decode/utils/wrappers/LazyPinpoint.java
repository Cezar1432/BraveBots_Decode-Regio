package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(
        xmlTag = "LazyPinpoint",
        name = "Cached Pinpoint"
)
public class LazyPinpoint extends GoBildaPinpointDriver implements HardwareDevice {
    public LazyPinpoint(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }
    int freq= 20;
    double last;
    public void setFreq(int freq){
        this.freq= freq;
        last= System.currentTimeMillis();

    }
    @Override
    public void update(ReadData data){
        if(System.currentTimeMillis()- last> freq)
        {
            last= System.currentTimeMillis();
            super.update(data);
        }
    }

    @Override
    public void update(){
        if(System.currentTimeMillis()- last> freq){
            last= System.currentTimeMillis();
            super.update();
        }
    }
}
