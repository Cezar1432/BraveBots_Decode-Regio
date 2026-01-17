package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains;


import org.firstinspires.ftc.teamcode.bravebots_decode.math.PDSFCoefficients;

public interface DrivetrainInterface {
    void drive(double s, double f, double r);
    void driveAuto(double s, double f, double r);
    void setCoefs(PDSFCoefficients c);
   // boolean ok= false;
}
