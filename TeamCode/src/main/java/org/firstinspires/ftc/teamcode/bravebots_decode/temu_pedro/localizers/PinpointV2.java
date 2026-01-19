package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;


public class PinpointV2 implements Localizer{
    @Override
    public void setStartingPose(Pose p) {

    }

    @Override
    public Pose getCurrentPosition() {
        return null;
    }

    @Override
    public Pose getRawPosition() {
        return null;
    }

    @Override
    public void updateGlide() {

    }

    @Override
    public double getPredictedX() {
        return 0;
    }

    @Override
    public double getPredictedY() {
        return 0;
    }

    @Override
    public Pose getActualPose() {
        return null;
    }

    @Override
    public void resetLocalizer() {

    }

    @Override
    public void update() {

    }

    @Override
    public void setOffsets(double x, double y, DistanceUnit unit) {

    }

    @Override
    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection forwardDirection, GoBildaPinpointDriver.EncoderDirection strafeDirection) {

    }

    @Override
    public double getXVelocity() {
        return 0;
    }

    @Override
    public double getYVelocity() {
        return 0;
    }

    @Override
    public double getHeadingVelocity() {
        return 0;
    }

    @Override
    public double getXYVelocity() {
        return 0;
    }
}
