package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.slew_rate_limiter;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {

    public double limiter, lastValue;
    public boolean firstRun;
    ElapsedTime timer;


    /**
     * joystick-urile iau valori de la 1 la -1, deci disatanta totala este "2".
     * @param limiter este cata astfel de distanta ii permiteti joystick-urilor sa faca pe secunda
     */
    public SlewRateLimiter(double limiter){
        this.limiter= limiter;
        this.lastValue= 0;
        firstRun= true;

    }



    public double calculate(double input) {

        if (!firstRun) {
            double deltaTime = timer.seconds();
            timer.reset();
            double maxChange = deltaTime * limiter;
            if (Math.abs(input - lastValue) > maxChange)
                lastValue += maxChange * Math.signum(input - lastValue);
            return lastValue;

        }
        timer= new ElapsedTime();
        firstRun = false;
        return input;

    }

    public void setLimiter(double limiter){
        this.limiter= limiter;
    }

}
