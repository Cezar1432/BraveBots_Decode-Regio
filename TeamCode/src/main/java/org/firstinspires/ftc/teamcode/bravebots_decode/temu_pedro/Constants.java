package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    public static double pStrafe= 0.02, dStrafe= 0.0006, secondaryPStrafe= 0.02, secondaryDStrafe= 0.0006;
    public static double pForward= 0.02, dForward= 0.0006, secondaryPForward= 0.02, secondaryDForward= 0.0006;
    public static double pHeading=  0.2 , dHeading=0.002, secondaryPHeading= 0.1, secondaryDHeading= 0.001;
    public static boolean useSecondaryStrafe= true, useSecondaryForward= true, useSecondaryHeading= true;
    public static double strafeThreshold=3, forwardThreshold=3, headingThreshold= 0.1;
    public static PIDFController strafe= new PIDFController(0.01,0,0.00001,0);
    public static PIDFController forward= new PIDFController(0.01,0,0.00001,0);
    public static PIDFController heading= new PIDFController(0.0003, 0, 0.0000003,0);
    //public static PIDFController heading= new PIDFController(3.2,0,0.23,0);
    public static PIDFController secondaryStrafe= new PIDFController(0.14,0,0.015,0);
    public static PIDFController secondaryForward= new PIDFController(0.08,0,0.00001,0);
    public static PIDFController secondaryHeading= new PIDFController(2,0,0.15,0);
    public static double filterParameter = 0.8;
    public static double xDeceleration = 1003716000, yDeceleration = 1003716000;
    public volatile static long threadUpdatePeriod= 1;
    public static double headingConstraint= 0.05;
    public static volatile double distanceConstraint= 0.5;
    public static double XYVelocityConstraint= 1;
    public static double headingVelocityConstraint= 0.1;






}
