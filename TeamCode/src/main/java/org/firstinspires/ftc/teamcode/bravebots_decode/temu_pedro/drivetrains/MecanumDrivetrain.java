package org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.PDSFCoefficients;



public class MecanumDrivetrain implements DrivetrainInterface {

    public DcMotorEx leftFront, leftBack, rightFront, rightBack;


    public MecanumDrivetrain(Robot robot){

        leftBack= robot.leftBack;
        leftFront= robot.leftFront;
        rightBack= robot.rightBack;
        rightFront= robot.rightFront;

    }




    public enum Actions{
        CLIMB, DRIVE;
    }
    Actions action= Actions.DRIVE;

    public void changeStatus(Actions action){
        this.action= action;
    }
    public Actions getAction(){
        return action;
    }


    @Override
    public void update(double s, double f, double r) {

    }

    //    public void drive(double x, double y, double r){
//        if(Controls.state== ControlStates.BASKET || Controls.state == ControlStates.SPECIMEN_PLACE)
//            r*=-0.7;
//
//        x*= -1;
//
//        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
//        double fl = (y + x + r) / d;
//        double bl = (y - x + r) / d;
//        double fr = (y - x - r) / d;
//        double br = (y + x - r) / d;
//
//        leftFront.setPower(fl);
//        leftBack.setPower(bl);
//        rightBack.setPower(br);
//        rightFront.setPower(fr);
//    }
    public void updateAuto(double s, double f, double r){
        s*= -1;
        r*=-1;
        //  f*=-1;
        r*= 1.1;
        double d= Math.max((Math.abs(s)+ Math.abs(f)+ Math.abs(r)), 1);

        double fl = (f + s + r) / d;
        double bl = (f - s + r) / d;
        double fr = (f - s - r) / d;
        double br = (f + s - r) / d;

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightBack.setPower(br);
        rightFront.setPower(fr);

    }


    public void setCoefs(PDSFCoefficients c) {

    }



    public void climb(double power){
        rightFront.setPower(power);
        leftBack.setPower(power);
    }


}
