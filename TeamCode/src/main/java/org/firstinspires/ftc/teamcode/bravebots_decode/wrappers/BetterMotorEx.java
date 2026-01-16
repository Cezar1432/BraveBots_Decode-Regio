package org.firstinspires.ftc.teamcode.bravebots_decode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.bravebots_decode.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.bravebots_decode.math.PDSFController;


@DeviceProperties(
        xmlTag = "betterMotorEx",
        name = "Better Motor Ex",
        description = "generic motor",
        builtIn = false
)
@MotorType(
        ticksPerRev = 1550,
        gearing = 55,
        maxRPM = 6000,
        achieveableMaxRPMFraction = 1.0,
        orientation = Rotation.CCW
)
public class BetterMotorEx extends DcMotorImplEx implements DcMotorEx, HardwareDevice {

    double ticksPerRevolution= 0;
    ElapsedTime time;
    long startTime;
    public BetterMotorEx(DcMotorController controller, int portNumber, int ticksPerRevolution) {
        super(controller, portNumber);
        this.ticksPerRevolution= ticksPerRevolution;
        startTime= System.nanoTime();
        time= new ElapsedTime();

    }public BetterMotorEx(DcMotorController controller, int portNumber) {
        super(controller, portNumber);

        time= new ElapsedTime();//aaa

    }

    public enum RunMode{
        RUN, PID, VELOCITY_PID
    }
    RunMode runMode= RunMode.RUN;
    double velocity= 0;
    private double targetPos= 0;
    @Override
    public void setPower(double power){
        super.setPower(power);
        lastPower= 0;
        runMode= RunMode.RUN;
    }


    public double getRPM(){

        return RPM;
    }


    double BULLSHIT_ERROR= 1.2;
    public void setRPM(double rpm){
        if(ticksPerRevolution== 0)
            throw new RuntimeException("Ticks rer revolution not set!");
        targetRPM= rpm;
        runMode= RunMode.VELOCITY_PID;
    }
    double cnt  = 0,sum  = 0, avg;

    public PDSFController velController= new PDSFController(0,0,0,0);
    public PDSFController pidController= new PDSFController(0,0,0,0);

    double targetRPM;

    public void setVelocityCoefficients(double p, double d){
        velController.setCoefficients(new PDSFCoefficients(p,d,0,0));
    }
    public void setPIDControllerCoefficients(double p, double d, double s, double f){
        pidController.setCoefficients(new PDSFCoefficients(p,d,s,f));
    }



    public void setTargetPosition(int targetPosition){
        runMode= RunMode.PID;
        this.targetPosition= targetPosition;
    }
    int reversed = 1;
    public enum Direction{
        FORWARD, BACKWARDS
    }
    public void changeDirection(Direction dir){
        if(dir== Direction.FORWARD)
            reversed= 1;
        else
            reversed= -1;
    }

    double lastTime = 0;
    int targetPosition = 0;

    double RPM= 0;
    double lastPower= 0;
    double lastTicks= 0;
    public void update(){


        double deltaTime = System.nanoTime()- lastTime;
        lastTime= System.nanoTime();
        deltaTime/= 1e9;

        double currentTicks= getCurrentPosition();
        double deltaTicks = currentTicks-  lastTicks;
        lastTicks= currentTicks;
        velocity= deltaTicks/deltaTime;
        double deltaRotations= deltaTicks / ticksPerRevolution;
        RPM= deltaRotations/ deltaTime;
        RPM= RPM* 60;


//        double currentTicks= getCurrentPosition();
//        long  currentTime=(System.nanoTime()- startTime)/1e9;
//        if(currentTime- lastTime> 1e9){
//            double deltaTime= currentTime- lastTime;
//            lastTime= currentTime;
//            double deltaTicks= currentTicks- lastTicks;
//            lastTicks= currentTicks;
//            double deltaRotations= deltaTicks/ ticksPerRevolution;
//            RPM= deltaRotations/(deltaTime * 1e9);
//            RPM*= 60;
//        }
        if(runMode== RunMode.PID){
            super.setPower(pidController.calculate(getCurrentPosition(), targetPosition));
            lastPower= 0;
        }
        else
        if(runMode==  RunMode.VELOCITY_PID){
            double power= lastPower+ velController.calculate(RPM, targetRPM);
            super.setPower(power);
            lastPower= power;
        }


    }
    double maxRPM= 0;
    public void setMaxRPM(double maxRPM){
        this.maxRPM= maxRPM;
    }

    double ticksThreshold= 2000;
    public void setTicksThreshold(double ticksThreshold){
        this.ticksThreshold = ticksThreshold;
    }

    double RPMThreshold= 100;
    public void RPMThreshold(double RPMThreshold){
        this.RPMThreshold= RPMThreshold;
    }

    @Override
    public boolean isBusy(){
        if(runMode== RunMode.PID)
            return Math.abs(targetPos- getCurrentPosition())< ticksThreshold;

        return Math.abs(targetRPM- RPM)< RPMThreshold;
    }

    public void resetEncoder(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}