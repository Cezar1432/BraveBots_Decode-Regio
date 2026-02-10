package org.firstinspires.ftc.teamcode.bravebots_decode.op_modes;

import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;

import kotlin.math.UMathKt;

public class Poses {

         public static Pose startPose= new Pose(110, 135.2, Math.toRadians(-90));
         public static Pose closeShootPose= new Pose(85.33748055987559, 82.5816485225506, Math.toRadians(-45));
         public static Pose secondSpikes = new Pose(103.02, 59, Math.toRadians(0));
         public static Pose secondSpikesCollect = new Pose(134, 59, Math.toRadians(0));
         public static Pose gateOpenPose= new Pose(132, 62.85, Math.toRadians(27));
         public static Pose intermediateGateOpen= new Pose(103, 60.8, Math.toRadians(30));
        // public static Pose firstSpikes= new Pose(103, 83.25, Math.toRadians(0));
         public static Pose firstSpikesCollect = new Pose(125.44, 83.7);

         public static Pose farStartPose = new Pose(87.3, 8.64,Math.toRadians(0));
         public static Pose thirdSpike= new Pose(100.56, 37.44,Math.toRadians(0));
         public static Pose thirdSpikesCollect= new Pose(136.90, 36.23,Math.toRadians(0));
         public static Pose humanPlayerCollect= new Pose(135.26, 10,Math.toRadians(0));
         public static Pose farLeave= new Pose(106.15, 11.59,Math.toRadians(0));

        // public static Pose humanPlayer2= new Pose(135.87, 12.941, Math.toRadians(20));
        public static Pose humanPlayer2= humanPlayerCollect;
         public static Pose farShootPose= new Pose(84.42003110419908, 12.716827371695189, Math.toRadians(0));

         public static Pose gateCollect= new Pose(130.7993779160187, 50.48367029548996, Math.toRadians(60));

}
