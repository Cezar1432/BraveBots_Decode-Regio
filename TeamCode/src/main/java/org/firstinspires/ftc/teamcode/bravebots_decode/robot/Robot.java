package org.firstinspires.ftc.teamcode.bravebots_decode.robot;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterMotor;

import java.util.List;

public class Robot {

    public GoBildaPinpointDriver odo;


    public BetterMotor leftFront, rightFront, leftBack, rightBack;
    public DcMotorController controlHubMotors, expansionHubMotors;
    public ServoController controlHubServos, expansionHubServos;
    public BetterCRServo fl, bl, fr, br;
    public AnalogInputController expansionHubAnalogInputController, controlHubAnalogInputController;
    public Alliance a;
    public HardwareMap hm;
    public Telemetry t;
    List<LynxModule> hubs;
    LynxModule controlHub, expansionHub;
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
        rightFront= new BetterMotor(controlHubMotors, 1);
        rightBack= new BetterMotor(controlHubMotors, 0);
    }

    public void initializeServos(){
        fl= new BetterCRServo(controlHubServos, 3, expansionHubAnalogInputController, 1);
        bl= new BetterCRServo(controlHubServos, 2,  expansionHubAnalogInputController, 0);
        fr= new BetterCRServo(controlHubServos, 1, controlHubAnalogInputController, 0);
        br= new BetterCRServo(controlHubServos, 0, controlHubAnalogInputController, 1);
    }

    public void initializeRest(){
        odo= hm.get(GoBildaPinpointDriver.class, "nigg");
        odo.setOffsets(11.1, -5, DistanceUnit.CM);
    }
    public void initialize(){

        instance= t== null? new Robot(this.hm, this.a) : new Robot(this.hm, this.t, this.a);
        hubs= hm.getAll(LynxModule.class);
        controlHub= hubs.get(0);
        expansionHub= hubs.get(1);
        initializeControllers();
        initializeMotors();
        initializeServos();
    }




}
