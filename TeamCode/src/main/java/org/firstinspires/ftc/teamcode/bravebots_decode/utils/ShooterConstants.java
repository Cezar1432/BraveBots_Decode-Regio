package org.firstinspires.ftc.teamcode.bravebots_decode.utils;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

public class ShooterConstants {
    public static Pose GOAL_RED_POS = new Pose(138,138);
    //public static Pose GOAL_BLUE_POS = GOAL_RED_POS.mirror();
    public static double SCORE_HEIGHT = .69;
    public static double SCORE_ANGLE = Math.toRadians(-30);
    public static double PASS_THROUGH_POINT_RADIUS = 0.33;
    public static double HOOD_MIN_ANGLE = Math.toRadians(16.3);
    public static double HOOD_MAX_ANGLE = Math.toRadians(47.5);
    public static double HOOD_MIN_POS= 0.86;
    public static double HOOD_MAX_POS = 0.17;
}
