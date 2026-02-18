package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;


@DigitalIoDeviceType
@DeviceProperties(xmlTag = "BreakBeam", name= "Break Beam")
public class BreakBeam extends DigitalChannelImpl implements HardwareDevice {
    /**
     * Constructor
     *
     * @param controller Digital channel controller this channel is attached to
     * @param channel  channel on the digital channel controller
     */
    public BreakBeam(DigitalChannelController controller, int channel) {
        super(controller, channel);
    }
    public enum Status{
        CLEAR, BROKEN
    }
    Status status= Status.CLEAR;
    public boolean isBeamBroken(){
        return !this.getState();
    }
    public Status getBeamState(){

        status= isBeamBroken() ? Status.BROKEN : Status.CLEAR;
        return status;
    }
}