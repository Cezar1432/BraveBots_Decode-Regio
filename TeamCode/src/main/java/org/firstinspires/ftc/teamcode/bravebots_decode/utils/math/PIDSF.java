package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math;

import com.arcrobotics.ftclib.controller.PIDFController;

public class PIDSF extends PIDFController {

    double ks;
    public PIDSF(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd, kf);
    }
    public PIDSF(double kp, double ki, double kd, double kf, double ks) {
        super(kp, ki, kd, kf);
        this.ks= ks;
    }
    public double calculate(double current, double target){
        return super.calculate(current, target) + ks * Math.signum(current-target);
    }
}
