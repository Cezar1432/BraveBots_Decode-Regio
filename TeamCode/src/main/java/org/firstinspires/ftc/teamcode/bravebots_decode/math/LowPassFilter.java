package org.firstinspires.ftc.teamcode.bravebots_decode.math;

public class LowPassFilter {
    double filterParameter;
    double lastValue;
    public LowPassFilter(double filterParameter, double initialValue){
        this.filterParameter= filterParameter;
        this.lastValue= initialValue;

    }

    public double getValue(double rawValue){
        double newValue= lastValue+ filterParameter* (rawValue- lastValue);
        this.lastValue= newValue;
        return newValue;
    }
}
