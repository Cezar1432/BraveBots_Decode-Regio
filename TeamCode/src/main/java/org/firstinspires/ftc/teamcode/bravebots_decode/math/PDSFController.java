package org.firstinspires.ftc.teamcode.bravebots_decode.math;


import com.arcrobotics.ftclib.controller.PIDFController;

public class PDSFController extends PIDFController {
    public double kS= 0;
    double kP, kF, kD;
    public PDSFController(double kp, double kd, double ks, double kf) {
        super(kp, 0 , kd, kf);
    }
    public PDSFController(double kp, double ki, double kd, double kf, double ks) {
        super(kp, ki, kd, kf);
        this.kS= ks;
    }
    public PDSFController(PDSFCoefficients coefs){
        super(coefs.p, 0, coefs.d, coefs.f);
    }
    public PDSFController(double kp,  double kd, double kf) {
        super(kp, 0, kd, kf);
    }
    public double ksOutput;

    public double sign= 0;
    public double calculate(int current, int target){

        sign=Math.signum(target - current);
        ksOutput= kS;


        return super.calculate(current, target)+ksOutput;
    }

    public double calculate(double error){
        sign= Math.signum(error);
        ksOutput= kS;
        return super.calculate(error)+ ksOutput;
    }
    public void setCoefficients(double kP, double kD, double kF){

        this.kP= kP;
        this.kD= kD;
        this.kF= kF;
        super.setPIDF(kP, 0, kD, kF);

    }
    public void setCoefficients(double kP, double kD, double kF, double kS){
        this.kP= kP;
        this.kD= kD;
        this.kF= kF;
        this.kS= kS;
        super.setPIDF(kP, 0, kD, kF);

    }
    public void setCoefficients(PDSFCoefficients coefs){
        this.kP= coefs.p;
        this.kD= coefs.d;
        this.kS= coefs.s;
        this.kF= coefs.f;
        super.setPIDF(kP, 0, kD, kF);

    }

}
