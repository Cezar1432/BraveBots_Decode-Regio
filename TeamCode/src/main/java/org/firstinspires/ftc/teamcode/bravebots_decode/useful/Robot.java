package org.firstinspires.ftc.teamcode.bravebots_decode.useful;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bravebots_decode.wrappers.BetterCRServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.wrappers.BetterMotor;

public class Robot {

    public GoBildaPinpointDriver odo;


    public BetterMotor leftFront, rightFront, leftBack, rightBack;
    public DcMotorController controlHubMotors, expansionHubMotors;
    public ServoController controlHubServos, expansionHubServos;
    public BetterCRServo fl, bl, fr, br;
    public AnalogInputController expansionHubAnalogInputController, controlHubAnalogInputController;
    public static Alliance a= Alliance.BLUE;
    public HardwareMap hm;
    public Telemetry t;

    public Robot(HardwareMap hm, Telemetry t, Alliance a){
        this.hm= hm;
        this.t= t;
        Robot.a= a;
    }
    public Robot(HardwareMap hm, Alliance a){
        this.hm= hm;

        Robot.a= a;
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
        leftBack= new BetterMotor(expansionHubMotors, 0);
        leftFront= new BetterMotor(expansionHubMotors, 1);
        rightFront= new BetterMotor(controlHubMotors, 1);
        rightBack= new BetterMotor(controlHubMotors, 0);
    }

    public void initializeServos(){
        fl= new BetterCRServo(expansionHubServos, 0, expansionHubAnalogInputController, 1);
        bl= new BetterCRServo(expansionHubServos, 1,  expansionHubAnalogInputController, 0);
        fr= new BetterCRServo(controlHubServos, 1, controlHubAnalogInputController, 0);
        br= new BetterCRServo(controlHubServos, 0, controlHubAnalogInputController, 1);
    }

    public void initialize(){
        initializeControllers();
        initializeMotors();
        initializeServos();
    }




}
