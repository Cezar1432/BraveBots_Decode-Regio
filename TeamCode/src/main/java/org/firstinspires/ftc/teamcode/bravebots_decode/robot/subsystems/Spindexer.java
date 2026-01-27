package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers.BetterServo;

import java.util.LinkedList;
import java.util.List;

public class Spindexer{

     public static BetterServo s1, s2;

     public static void shootRandom(){
//         s1.setMaxDegrees(1100);
//         s2.setMaxDegrees(1100);
         //s1.turn( 360);
         s1.setPosition(1);
          //  s2.setPosition(s1.getPosition());
     }

     public static void turnBack(){
         s1.setPosition(0);
       //  s2.turnToAngle(s2.getPosition());
     }




}
