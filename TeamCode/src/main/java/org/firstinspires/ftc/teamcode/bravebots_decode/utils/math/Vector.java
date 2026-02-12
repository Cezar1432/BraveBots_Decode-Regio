package org.firstinspires.ftc.teamcode.bravebots_decode.utils.math;

public class Vector {

    public double magnitude, theta;
    public double xComponent, yComponent;

    public enum Type{
        CARTESIAN, POLAR
    }
    Type type;
    public Vector(double a, double b, Type type){
        if(type== Type.CARTESIAN)
        {
            xComponent= a;
            yComponent= b;
            magnitude= Math.hypot(a,b);
            theta= Math.atan2(b,a);
        }
        else{
            if(a< 0)
                theta= MathStuff.normalizeRadians(b+ Math.PI);
            else
                theta= MathStuff.normalizeRadians(b);
            magnitude= Math.abs(a);
            xComponent= magnitude * Math.cos(theta);
            yComponent= magnitude * Math.sin(theta);
        }
        this.type= type;
    }


    public double getXComponent(){
       return this.xComponent;
    }
    public double getYComponent(){
        return this.yComponent;
    }

    public double getMagnitude(){
        return this.magnitude;
    }
    public double getTheta(){
        return this.theta;
    }
    public Vector rotateBy(double rad){

        double x= Math.cos(rad) * getXComponent() - Math.sin(rad) * getYComponent();
        double y= Math.sin(rad) * getXComponent() + Math.cos(rad) * getYComponent();

        return new Vector(x, y, Type.CARTESIAN);
    }

}


