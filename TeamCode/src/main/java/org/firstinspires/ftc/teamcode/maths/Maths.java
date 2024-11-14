package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Maths {

    //normalizes the angle given
    public static double angleWrap(double wrap) {

        while(wrap <= -180) {
            wrap += 360;
        }
        while(wrap > 180) {
            wrap -= 360;
        }
        return wrap;
    }

    //replaces turning a module by 180 degrees with reversing motor power.
    public static double[] efficientTurn(double reference,double state,double power) {
        double error = reference-state;

        while(error > 90) {
            power *= -1;
            reference -= 180;
            error = reference - state;
        }
        while(error<-90) {
            power *= -1;
            reference += 180;
            error = reference - state;
        }

        return new double[] {reference,power};
    }

    public static boolean dynamicTurn(double error){ return Math.abs(error) > 90; }

    //converts two degrees of freedom into a differential
    public static double[] diffyConvert(double rotate, double translate){
        double m1 = rotate + translate;
        double m2 = rotate - translate;
        double maxi = Math.max(Math.abs(m1),Math.abs(m2));
        if(maxi > 1){
            m1 /= Math.abs(maxi);
            m2 /= Math.abs(maxi);
        }
        return new double[] {m1,m2};
    }

    //math for detecting when an absolute encoder has wrapped around
    public static double modWrap(double state, double wrap, double last, double ratio){
        double delta = state - last;

        if (delta > 180) wrap += 1;
        if (delta < -180) wrap += 1;
        if (wrap > ratio - 1) wrap = 0;
        if (wrap == 0) return state / ratio;
        return 360 / (wrap + 1) + state / ratio;
    }

    public static double wappa(double state, double wrap, double last, double ratio){
        double delta = state - last;

        if (delta > 180) wrap += 1;
        if (delta < -180) wrap += 1;
        if (wrap > ratio - 1) wrap = 0;
        if (wrap == 0) return state / ratio;
        return 360 / (wrap + 1) + state / ratio;
    }

    public static boolean epsilonEquals(double state, double equals, double thresh){
        return Math.abs(state - equals) < thresh;
    }

    public static Vector2d[] pointListToVectorList(double[] coordinateList) {
        return new Vector2d[]{
                new Vector2d(coordinateList[0], coordinateList[1]),
                new Vector2d(coordinateList[2], coordinateList[3]),
                new Vector2d(coordinateList[4], coordinateList[5]),
                new Vector2d(coordinateList[6], coordinateList[7]),
                new Vector2d(coordinateList[8], coordinateList[9]),
                new Vector2d(coordinateList[10], coordinateList[11]),
                new Vector2d(coordinateList[12], coordinateList[13]),
                new Vector2d(coordinateList[14], coordinateList[15])
        };
    }

    public static double magnitudeOf(Vector2d vec) { return Math.sqrt(Math.pow(vec.getX(),2)+Math.pow(vec.getY(),2)); }

    public static double distanceBetween(Vector2d a, Vector2d b) { return magnitudeOf(a.minus(b)); }

    public static Vector2d rotateVectorBy(Vector2d vec, double radians) {
        double x = vec.getX() * Math.cos(radians) - vec.getY() * Math.sin(radians);
        double y = vec.getX() * Math.sin(radians) + vec.getY() * Math.cos(radians);
        return new Vector2d(x,y);
    }

    public static double angleOf(Vector2d vec) { return AngleUnit.normalizeRadians(Math.atan2(vec.getY(),vec.getX())); }

    public static Vector2d interpolateBetweenVectors(Vector2d start, Vector2d end, double interpolator){
        double m = (start.getY() - end.getY()) / (start.getX() - end.getX());
        double b = -(m*start.getX()) + start.getY();

        return new Vector2d((interpolator - b) / m, interpolator);
    }
    
    public static double averageOf(double a, double b) {
        return (a+b)/2;
    }

    public static double crossOf(Vector2d a, Vector2d b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }

    public static Vector2d swapXYOf(Vector2d vec) {
        return new Vector2d(vec.getY(), vec.getX());
    }
}
