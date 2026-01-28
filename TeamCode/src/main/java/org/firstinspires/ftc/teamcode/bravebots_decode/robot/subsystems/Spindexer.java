package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;

import java.util.LinkedList;
import java.util.List;

public class Spindexer{

     public static BetterServo s1, s2;

     public static void shootRandom(){
//         s1.setMaxDegrees(1100);
//         s2.setMaxDegrees(1100);
         s1.turn( 360);
          //  s2.setPosition(s1.getPosition());
     }

     public enum Slots{
         SLOT_1(0,0), SLOT_2(0,0), SLOT_3(0,0);

         double frontPose, shootPose;
         Slots(double frontPose, double shootPose){
             this.frontPose= frontPose;
             this.shootPose= shootPose;
         }

     }

     public static void turnTo(Slots slot){
         s1.setPosition(slot.frontPose);
     }
     public static void shootSlot(Slots slot){
         s1.setPosition(slot.shootPose);
     }
     public static void turnBack(){
         turnTo(Slots.SLOT_1);
     }

     public static void turnManuallyToRight(){
         s1.turn(120);
     }
     public static void turnManuallyToLeft(){
         s1.turn(-120);
     }




}
