package org.firstinspires.ftc.teamcode.bravebots_decode.robot;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

import javax.microedition.khronos.egl.EGL;

public class Robot {

    public GoBildaPinpointDriver odo;


    public volatile BetterMotor leftFront, rightFront, leftBack, rightBack, turret, intake;
    public BetterMotorEx shooter;
    public DcMotorController controlHubMotors, expansionHubMotors;
    public ServoController controlHubServos, expansionHubServos;
    public volatile BetterCRServo fl, bl, fr, br;
    public BetterServo indexer1, indexer2, hood;
    public AnalogInputController expansionHubAnalogInputController, controlHubAnalogInputController;
    public Alliance a;
    public HardwareMap hm;
    public Telemetry t;
    List<LynxModule> hubs;
    LynxModule controlHub, expansionHub;
    OpenCvCamera camera;
    Limelight3A ll;
    static Robot instance;

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



    }
    public void initializeMotors(){

        leftBack= new BetterMotor(expansionHubMotors, 1);
        leftFront= new BetterMotor(expansionHubMotors, 0);
        rightFront= new BetterMotor(controlHubMotors, 0);
        rightBack= new BetterMotor(controlHubMotors, 1);
        turret= new BetterMotor(controlHubMotors, 2);
        intake= new BetterMotor(expansionHubMotors, 2);
        shooter= new BetterMotorEx(expansionHubMotors, 3);

    }
    private void assignHardware(){
        Intake.motor= intake;
        Shooter.m= shooter;
        Shooter.s= hood;
        Spindexer.s1= indexer1;
        Turret.m= turret;
    }

    public void initializeServos(){

        fl= new BetterCRServo(expansionHubServos, 0, expansionHubAnalogInputController, 1);
        bl= new BetterCRServo(expansionHubServos, 1,  expansionHubAnalogInputController, 0);
        fr= new BetterCRServo(controlHubServos, 0, controlHubAnalogInputController, 1);
        br= new BetterCRServo(controlHubServos, 1, controlHubAnalogInputController, 0);
        indexer1= new BetterServo(controlHubServos, 2);
        indexer1.setMaxDegrees(1100);
        hood= new BetterServo(expansionHubServos, 2);

    }

    public void initializeRest(){
        odo= hm.get(GoBildaPinpointDriver.class, "nigg");
        odo.setOffsets(11.1, -5, DistanceUnit.CM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //ll= hm.get(Limelight3A.class, "ll");
        int pipeline= a== Alliance.RED ? 2 : 9;
        //ll.pipelineSwitch(pipeline);
        //WebcamName name= hm.get(WebcamName.class, "camera");
        //camera= OpenCvCameraFactory.getInstance().createWebcam(name);

    }
    public double robotHeading= 0;
    public void initialize(){

        instance= t== null? new Robot(this.hm, this.a) : new Robot(this.hm, this.t, this.a);
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
    ElapsedTime timer;
    public void update(){
        if(timer== null)
            timer= new ElapsedTime();
        hubs.forEach(LynxModule::clearBulkCache);
        //hubs.forEach(LynxModule::close);


        odo.update();
        robotHeading= odo.getHeading(AngleUnit.DEGREES);

    }




}
