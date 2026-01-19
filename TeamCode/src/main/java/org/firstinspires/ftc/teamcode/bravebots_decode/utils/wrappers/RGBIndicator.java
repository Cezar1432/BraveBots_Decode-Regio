package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

@ServoType(
        flavor = ServoFlavor.CUSTOM
)
@DeviceProperties(
        xmlTag = "rgbIndicator"
        ,
        name = "RGB Indicator"
)
public class RGBIndicator extends ServoImpl implements Servo, HardwareDevice {
    public RGBIndicator(ServoControllerEx controller, int portNumber) {
        super(controller, portNumber);
    }

    public enum Colors{
        BLUE(0.6028), WHITE(0.8367), RED(0.2961), GREEN(0.4578), PURPLE(0.7078);

        double pos;
        Colors(double pos){
            this.pos= pos;
        }
    }

    public void setColor(Colors color){
        setPosition(color.pos);
    }

    public void setColor(double pos){
        setPosition(pos);
    }



}