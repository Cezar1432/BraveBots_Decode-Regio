package org.firstinspires.ftc.teamcode.bravebots_decode.p2p_test;


import com.arcrobotics.ftclib.controller.PIDFController;

public class Constants {

    public static double pStrafe= 0.14, dStrafe= 0.015, secondaryPStrafe= 0.1, secondaryDStrafe= 0.01;
    public static double pForward= 0.08, dForward= 0.00001, secondaryPForward= 0.08, secondaryDForward= 0.00001;
    public static double pHeading=  2.2 , dHeading=0.15, secondaryPHeading= 0, secondaryDHeading= 0;
    public static boolean useSecondaryStrafe= false, useSecondaryForward= true, useSecondaryHeading= false;
    public static double strafeThreshold=3, forwardThreshold=3, headingThreshold= 0.1;
    public static PIDFController strafe= new PIDFController(0.01,0,0.00001,0);
    public static PIDFController forward= new PIDFController(0.01,0,0.00001,0);
    public static PIDFController heading= new PIDFController(2,0,0.15,0);
    public static PIDFController secondaryStrafe= new PIDFController(0.14,0,0.015,0);
    public static PIDFController secondaryForward= new PIDFController(0.08,0,0.00001,0);
    public static PIDFController secondaryHeading= new PIDFController(2,0,0.15,0);
    public static double filterParameter = 0.8;
    public static double xDeceleration = 3 * 18000 * 25.4, yDeceleration = 3 * 22500 * 25.4;
    public volatile static long threadUpdatePeriod= 1;
    public static double headingConstraint= 0.05;
    public static volatile double distanceConstraint= 0.5;
    public static double XYVelocityConstraint= 1;
    public static double headingVelocityConstraint= 0.1;






}
