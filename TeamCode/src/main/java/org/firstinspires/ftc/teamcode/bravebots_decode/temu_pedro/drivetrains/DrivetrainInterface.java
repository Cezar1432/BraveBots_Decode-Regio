package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains;


import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;

public interface DrivetrainInterface {
    void update(double s, double f, double r);
    void updateAuto(double s, double f, double r);
    void setCoefs(PDSFCoefficients c);
   // boolean ok= false;
}
