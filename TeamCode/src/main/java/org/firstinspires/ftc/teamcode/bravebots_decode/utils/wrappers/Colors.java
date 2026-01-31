package org.firstinspires.ftc.teamcode.bravebots_decode.utils.wrappers;

public class Colors {

    public static class Color{
        public double r,g,b;
        public Color(double r, double g, double b){
            this.r= r;
            this.g=g;
            this.b=b;
        }
    }

    public enum Balls{

        PURPLE(new Color(180, 0, 180), "Purple")
        ,
        GREEN(new Color(0, 255, 0), "Green")
        ,
        NONE(new Color(0,0,0), "None");

        final Color color;
        final String name;

        Balls(Color color, String name){
            this.color= color;
            this.name= name;
        }
        public String getName(){
            return name;
        }

    }

    public static double getColorDistance(Color c1, Color c2){
        double rD= c1.r- c2.r;
        double gD= c1.g- c2.g;
        double bD= c1.b- c2.b;
        return Math.cbrt(rD*rD + gD*gD + bD*bD);
    }

    public static Balls getColor(Color c){
        Balls finalColor= Balls.NONE;
        double diff= Double.POSITIVE_INFINITY;
        for(Balls b: Balls.values()){
            double locDiff= getColorDistance(b.color, c);
            if(locDiff< diff){
                diff= locDiff;
                finalColor= b;

            }
        }

        return finalColor;

    }
}
