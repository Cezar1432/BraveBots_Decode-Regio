package org.firstinspires.ftc.teamcode.bravebots_decode.wrappers.colorSensor;

public class Colors {

    public static class Color{
        public double r,g,b;
        public Color(double r, double g, double b){
            this.r= r;
            this.g= g;
            this.b= b;
        }
    }
    public enum Balls{


        COLOR1(new Color(0,0,0)),
        COLOR2(new Color(0,0,0)),
        NONE(new Color(0,0,0));


        Color color;
        Balls(Color color){
            this.color= color;
        }



    }
    public static double getColorDistance(Color color1, Color color2){

        double rDifference= color1.r- color2.r;
        double gDifference= color1.g- color2.g;
        double bDifference= color1.b- color2.b;

        return Math.cbrt(rDifference*rDifference + gDifference*gDifference + bDifference*bDifference);
    }

    public static Balls getColor(Color input){

        Balls color= Balls.NONE;
        double minDist= Double.MAX_VALUE;
        for(Balls sampleColor: Balls.values()){
            //if(sampleColor == SampleColors.NONE) continue;
            double dist= getColorDistance(input, sampleColor.color);
            if(dist< minDist){
                minDist= dist;
                color= sampleColor;
            }
        }
        return color;


    }
}
