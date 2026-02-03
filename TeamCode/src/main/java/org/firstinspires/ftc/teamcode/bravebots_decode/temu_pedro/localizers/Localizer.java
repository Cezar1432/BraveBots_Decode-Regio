package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;


public interface Localizer {
    void setStartingPose(Pose p);
    Pose getCurrentPosition();
    Pose getRawPosition();
    void updateGlide();
    double getPredictedX();
    double getPredictedY();
    Pose getActualPose();
    void resetLocalizer();
    Pose getPredictedPose();
    static double normalizeHeading(double h){
        while(h> Math.PI) h-= 2* Math.PI;
        while(h< -Math.PI) h+= 2* Math.PI;
        return h;
    }
    @Deprecated
    static double normalizeDegrees(double degrees){
        while(degrees> 180) degrees-= 360;
        while(degrees< -180) degrees+= 360;
        return degrees;
    }

    void update();
    void setOffsets(double x, double y, DistanceUnit unit);
    void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection forwardDirection, GoBildaPinpointDriver.EncoderDirection strafeDirection);
    double getXVelocity();
    double getYVelocity();
    double getHeadingVelocity();
    double getXYVelocity();
}
