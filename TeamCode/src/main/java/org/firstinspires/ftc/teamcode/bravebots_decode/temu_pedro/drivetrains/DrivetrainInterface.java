package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains;


import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;

import java.util.function.DoubleSupplier;

public interface DrivetrainInterface {
    DrivetrainInterface setSuppliers(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX);
    void update();
    void updateAuto(double s, double f, double r);
    DrivetrainInterface setCoefs(PDSFCoefficients c);
    DrivetrainInterface setTrackWidth(double trackWith);
    DrivetrainInterface setWheelBase(double wheelBase);
   // boolean ok= false;
}
