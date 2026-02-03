package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.view.inputmethod.InsertGesture;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.localizers.Localizer;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.BetterOpMode;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterGamepad;

@TeleOp
@Configurable
public class LocalizerTest extends BetterOpMode {

    public static double xOffset= 11.6, yOffset= -5.35;

    GoBildaPinpointDriver odo;
    @Override
    public void initialize() {
        odo= hardwareMap.get(GoBildaPinpointDriver.class, "nigg");
        odo.setOffsets(xOffset, yOffset, CM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS).whenPressed(()->odo.setOffsets(xOffset, yOffset, CM));

    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void activeLoop() {

        odo.update();
        telemetry.addData("x", odo.getPosX(INCH));
        telemetry.addData("y", odo.getPosY(INCH));
        telemetry.addData("heading", odo.getHeading(RADIANS));
        telemetry.update();
    }

    @Override
    public void init_start() {

    }

    @Override
    public void end() {

    }
}
