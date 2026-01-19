package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.bravebots_decode.op_modes.Alliance;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.Robot;
import org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems.Turret;

import java.util.Objects;

@Configurable
public class LimelightMath {
    public static Limelight3A ll;
    public static double goalAngle= Math.toRadians(54), xOffset= 0.0, yOffset= 0.0;
    //y = 31, x = 38

    static Robot robot= Robot.getInstance();
    public static LLResult getResults(){

        LLResult res= ll.getLatestResult();

        return res;
    }


    public static double getDistance(){
        double distance;
        if(robot.a == Alliance.RED) {
            distance = Math.hypot(1.8- Objects.requireNonNull(LimelightMath.getRobotPose()).x, LimelightMath.getRobotPose().y);
            return distance;
        }
        distance = Math.hypot(144 - LimelightMath.getRobotPose().x, LimelightMath.getRobotPose().y);
        return distance;
    }



    //    public static Pose getRobotPose(){
//        Position p = getResults().getCameraPoseTargetSpace().getPosition();
//        if(Robot.alliance== Robot.Alliance.RED)
//            return new Pose(-Math.sin(goalAngle)* p.z- Math.cos(goalAngle)*p.x + xOffset, -Math.cos(goalAngle)* p.z + Math.sin(goalAngle)* p.x + yOffset);
//
//        return new Pose(-Math.cos(goalAngle)* p.z+ Math.cos(goalAngle)*p.x + xOffset, -Math.cos(goalAngle)* p.z - Math.sin(goalAngle)* p.x + yOffset);
//
//    }
    public static Pose getRobotPose(){

        LLResult res= getResults();
        if(res!= null && res.isValid()){
            Pose3D pose= res.getBotpose();
            return new Pose(pose.getPosition().y, pose.getPosition().x, pose.getOrientation().getYaw());
        }
        return null;

    }


    public static double getLimelightAngle(){
        double angle;
        if(robot.a == Alliance.RED){
            angle = -Math.toDegrees(Math.atan2(LimelightMath.getRobotPose().x - xOffset, LimelightMath.getRobotPose().y - yOffset));
            return angle;
        }
        angle = -Math.toDegrees(Math.atan2(144 - LimelightMath.getRobotPose().x - xOffset, LimelightMath.getRobotPose().y - yOffset));
        return angle;
    }

    public static double getTurretAngle(){
        double angle;
        if(robot.a == Alliance.RED){
            angle = -Math.toDegrees(Math.atan2(LimelightMath.getRobotPose().x, LimelightMath.getRobotPose().y));
            return angle;
        }
        angle = -Math.toDegrees(Math.atan2(144 - LimelightMath.getRobotPose().x, LimelightMath.getRobotPose().y));
        return angle;
    }
    public static double angle= 0, RPM= 0;
    public static void updateShooterOutput(){


    }

    public static double getShooterAngle(){
        updateShooterOutput();
        return angle;

    }

    public static double getShooterRPM(){
        updateShooterOutput();
        return RPM;
    }
    public static double getLimelightUpdateAngle(){
        double chassisAngle= robot.odo.getHeading(AngleUnit.DEGREES);
        double llAngle= MathStuff.normalizeDegrees(chassisAngle- 180);
        llAngle= MathStuff.normalizeDegrees(llAngle+ Turret.getAngle());
        llAngle= MathStuff.normalizeDegrees(llAngle+ 90);
        return llAngle;
    }

    public static Pose getRobotPosition(){
        Pose llPose= getRobotPose();
        return new Pose();
    }


}
