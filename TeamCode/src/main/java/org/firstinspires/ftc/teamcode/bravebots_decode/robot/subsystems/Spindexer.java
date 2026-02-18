package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base.Scheduler;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterColorSensor;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.Colors;

import java.util.LinkedList;
import java.util.List;

public class Spindexer{

     public static BetterServo s1, s2;

     public static void setPosition(double pos){
         s1.setPosition(pos);
         s2.setPosition(pos);
     }
     public static void shootRandom(){
//         s1.setMaxDegrees(1100);
//         s2.setMaxDegrees(1100);
        setPosition(1);
          //  s2.setPosition(s31.getPosition());
     }

     public enum Slots{
         SLOT_1(0.4967,0), SLOT_2(.3739,0), SLOT_3(.2383  ,0),EJECT1(0.38,0),EJECT2(0.5117,0),EJECT3(0.6233,0);   //poz bune initiale
       //  SLOT_1(0.4494,0), SLOT_2(.311,0), SLOT_3(.215,0),EJECT1(0.38,0),EJECT2(0.5117,0),EJECT3(0.6233,0);
      // SLOT_1(0.4656,0), SLOT_2(.345,0), SLOT_3(.2217,0),EJECT1(0.38,0),EJECT2(0.5117,0),EJECT3(0.6233,0);

         final double frontPose, shootPose;
        // final double shootPose;
         Slots(double frontPose, double shootPose){
             this.frontPose= frontPose;
             this.shootPose= shootPose;
         }

     }

     static Scheduler s= new Scheduler();


     public static void turnTo(Slots slot){
         setPosition(slot.frontPose);
         currentSlot= slot;
     }
     public static void shootSlot(Slots slot){
         s1.setPosition(slot.shootPose);
     }
     public static void turnBack(){
         turnTo(Slots.SLOT_1);
     }

     public static double minimumTime= 350;
     public static double lastTime= 0;
     public static Slots currentSlot= Slots.SLOT_1;

     public static void turnManuallyToRight(){
         s1.turn(120);
         s2.turn(-120);
     }
     public static void turnManuallyToLeft(){
         s1.turn(-120);
         s2.turn(120);
     }

     public static boolean sorting= false;
     public static void setSorting(boolean set){
         sorting= set;
     }
     public static boolean testBoolean, isBroken;
     public static BetterColorSensor colorSensor;
    public static DigitalChannel breakBeam;

    public static boolean inSlot= false;

     public static double BallInDist= 10;
     public static void update(){
         if(Intake.intaking){
             if(sorting){
                 throw new IllegalArgumentException("nu stim sa sortam inca");
             }
             else{
                 double dist= colorSensor.getDistanceInCM();
                 testBoolean= dist< BallInDist;
                 isBroken = !breakBeam.getState();
                 if(System.currentTimeMillis()- lastTime> minimumTime && isBroken && currentSlot!= Slots.SLOT_3){

                     lastTime= System.currentTimeMillis();
                     if(currentSlot== Slots.SLOT_1)
                         currentSlot= Slots.SLOT_2;
                     else if(currentSlot== Slots.SLOT_2)
                         currentSlot= Slots.SLOT_3;

                     turnTo(currentSlot);
                 }
             }
         }
     }


}
