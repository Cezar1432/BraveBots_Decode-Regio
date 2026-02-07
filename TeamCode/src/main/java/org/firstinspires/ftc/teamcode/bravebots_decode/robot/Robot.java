package org.firstinspires.ftc.teamcode.bravebots_decode.robot;


import android.graphics.Path;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Constants;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.LimelightMath;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterColorSensor;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.EvenBetterServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.LazyPinpoint;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

public class Robot {

    public static LazyPinpoint odo;


    DcMotorControllerEx exExpansionHubMotors;
    public volatile BetterMotor leftFront, rightFront, leftBack, rightBack, turret;
    public BetterMotorEx intake;
    public DcMotorEx shooter, shooter2;
    public DcMotorController controlHubMotors, expansionHubMotors;
    public ServoController controlHubServos, expansionHubServos;
    public volatile BetterCRServo fl, bl, fr, br;
    public BetterServo indexer1, indexer2;
    public EvenBetterServo hood;
    public AnalogInputController expansionHubAnalogInputController, controlHubAnalogInputController;
    public static Alliance a= null;
    BetterColorSensor colorSensor;
    public HardwareMap hm;
    public Telemetry t;
    List<LynxModule> hubs;
    LynxModule controlHub, expansionHub;
    OpenCvCamera camera;
    public Limelight3A ll;
    static Robot instance;
    public static VoltageSensor voltageSensor;

    public static Robot getInstance(){
        return instance;
    }

    public Robot(HardwareMap hm, Telemetry t, Alliance a){
        this.hm= hm;
        this.t= t;
        this.a= a;
    }
    public Robot(HardwareMap hm, Alliance a){
        this.hm= hm;

        this.a= a;
    }


    public void initializeControllers(){

        controlHubMotors= hm.get(DcMotorController.class, "Control Hub");
        expansionHubMotors= hm.get(DcMotorController.class, "Expansion Hub 2");
        controlHubServos= hm.get(ServoController.class, "Control Hub");
        expansionHubServos= hm.get(ServoController.class, "Expansion Hub 2");
        controlHubAnalogInputController= hm.get(AnalogInputController.class, "Control Hub");
        expansionHubAnalogInputController= hm.get(AnalogInputController.class, "Expansion Hub 2");
        exExpansionHubMotors= hm.get(DcMotorControllerEx.class, "Expansion Hub 2");


    }
    public void initializeMotors(){

        leftBack= new BetterMotor(expansionHubMotors, 1).setCachingTolerance(.02);
        leftFront= new BetterMotor(expansionHubMotors, 0).setCachingTolerance(.02);
        rightFront= new BetterMotor(controlHubMotors, 0).setCachingTolerance(.02);
        rightBack= new BetterMotor(controlHubMotors, 1).setCachingTolerance(.02);
        turret= new BetterMotor(controlHubMotors, 2).setCachingTolerance(.02);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake= new BetterMotorEx(controlHubMotors, 3);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter= hm.get(DcMotorEx.class, "shooter");
        shooter2= hm.get(DcMotorEx.class, "shooter2");
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    private void assignHardware(){
        Intake.motor= intake;
        Shooter.motor1 = shooter;
        Shooter.motor2= shooter2;
        //Shooter.setCoefs();
        Shooter.s= hood;
        Spindexer.s1= indexer1;
        Spindexer.s2= indexer2;
        Spindexer.colorSensor= colorSensor;
        Turret.m= turret;
        LimelightMath.ll = ll;
        LimelightMath.robot= this;

    }

    public void initializeServos(){

        fl= new BetterCRServo(expansionHubServos, 0, expansionHubAnalogInputController, 1).setCachingTolerance(.02);
        bl= new BetterCRServo(expansionHubServos, 1,  expansionHubAnalogInputController, 0).setCachingTolerance(.02);
        fr= new BetterCRServo(controlHubServos, 0, controlHubAnalogInputController, 1).setCachingTolerance(.02);
        br= new BetterCRServo(controlHubServos, 1, controlHubAnalogInputController, 0).setCachingTolerance(.02);
        indexer1= new BetterServo(controlHubServos, 2).setMaxDegrees(1065);
        indexer2= new BetterServo(expansionHubServos, 3).setMaxDegrees(1065);
        hood= new EvenBetterServo(expansionHubServos, 2);

    }

    public void initializeRest(){
        //odo= hm.get(GoBildaPinpointDriver.class, "nigg");
        odo= hm.get(LazyPinpoint.class, "nigg");
        odo.setFreq(200);
        odo.setOffsets(Constants.xOffset, Constants.yOffset, DistanceUnit.CM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        colorSensor= hm.get(BetterColorSensor.class, "color sensor");


        ll= hm.get(Limelight3A.class, "ll");
        int pipeline= a== Alliance.RED ? 2 : 9;
        ll.pipelineSwitch(2);
        ll.setPollRateHz(100);
        ll.start();
        //WebcamName name= hm.get(WebcamName.class, "camera");
        //camera= OpenCvCameraFactory.getInstance().createWebcam(name);

    }
    public double updatedHeading= 0, robotHeading = 0;
    public void initialize(){

        //instance= t== null? new Robot(this.hm, this.a) : new Robot(this.hm, this.t, this.a);
        instance= this;
        hubs= hm.getAll(LynxModule.class);
        hubs.forEach((hub)-> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
     //   hubs.forEach(LynxModule::stopBlinking);

        controlHub= hubs.get(0);
        expansionHub= hubs.get(1);
//        for(LynxModule hub: hubs)
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        initializeControllers();
        initializeMotors();
        initializeServos();
        initializeRest();
        assignHardware();
    }


    public enum OpModeType{
        TELEOP, AUTO
    }
    OpModeType opModeType= OpModeType.TELEOP;
    public void setOpModeType(OpModeType type){
        opModeType= type;
    }
    ElapsedTime timer;
    public void update(){
        if(timer== null)
            timer= new ElapsedTime();
        hubs.forEach(LynxModule::clearBulkCache);
        //hubs.forEach(LynxModule::close);



        if(opModeType== OpModeType.TELEOP)
            odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        robotHeading = odo.getHeading(AngleUnit.DEGREES);

        // odo.update();

        if(Turret.getState()== Turret.State.TRACKING) {
            updatedHeading = LimelightMath.getLimelightUpdateAngle();
            ll.updateRobotOrientation(updatedHeading);
        }
    }

    public static boolean shooting= false;




}
