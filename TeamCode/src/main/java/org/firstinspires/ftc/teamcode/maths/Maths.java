package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

public class Maths {

    //normalizes the angle given
    public static double angleWrapDegrees(double deg) {
        double degrees = deg % 360;
        if (degrees >= 180.0) degrees -= 360.0;
        if (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    public static double angleWrapRadians(double rad) {
        double radians = rad % (2 * Math.PI);
        if (radians >= Math.PI) radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    //replaces turning a module by 180 degrees with reversing motor power.
    public static double[] efficientTurn(double reference,double state,double power) {
        double error = reference - state;

        while(error > 90) {
            power *= -1;
            reference -= 180;
            error = reference - state;
        }
        while(error < -90) {
            power *= -1;
            reference += 180;
            error = reference - state;
        }

        return new double[] {reference, power};
    }

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
    public static double absoluteBoundaryDetection(double state, double wrap, double last, double ratio){
        double delta = state - last;

        if (delta > 180) wrap += 1;
        if (delta < -180) wrap += 1;
        if (wrap > ratio - 1) wrap = 0;
        if (wrap == 0) return state / ratio;
        return 360 / (wrap + 1) + state / ratio;
    }

    public static boolean epsilonEquals(double state, double equals, double thresh) {
        return Math.abs(state - equals) < thresh;
    }

    public static boolean epsilonEquals(double state, double equals) {
        return Math.abs(state - equals) < 1e-6;
    }

    public static boolean pointDistance(Point a, Point b, double thresh) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2)) < thresh;
    }

    /**
     * solves for all of the real roots of a quadratic equation in the form of ax^2+bx+c
     * @param a ax^2
     * @param b bx
     * @param c c
     * @return all real roots
     */
    public static List<Double> solveRealQuadratic(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;
        List<Double> realSolutions = new ArrayList<>();
        if (Maths.epsilonEquals(discriminant, 0.0)) realSolutions.add(-b / (2 * a));
        else if (discriminant > 0.0) {
            realSolutions.add(-b + Math.sqrt(discriminant) / (2 * a));
            realSolutions.add(-b - Math.sqrt(discriminant) / (2 * a));
        }
        return realSolutions;
    }

    public static double calculateQuadratic(double A, double B, double C, double x) {
        return A * Math.pow(x, 2) + B * x + C;
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

    public static Vector2d rotateCartesianVectorBy(Vector2d vec, double radians) {
        double x = vec.getX() * Math.cos(radians) - vec.getY() * Math.sin(radians);
        double y = vec.getX() * Math.sin(radians) + vec.getY() * Math.cos(radians);
        return new Vector2d(x, y);
    }

    /**
     * Rotates a polar vector counterclockwise in radians
     */
    public static Vector2d rotatePolarVectorBy(Vector2d vec, double radians) {
        return new Vector2d(vec.getX(), vec.getY() + radians);
    }

    public static double angleOf(Vector2d vec) { return Maths.angleWrapRadians(Math.atan2(vec.getY(),vec.getX())); }

    public static Vector2d interpolateBetweenVectors(Vector2d start, Vector2d end, double interpolator){
        double m = (start.getY() - end.getY()) / (start.getX() - end.getX());
        double b = -(m * start.getX()) + start.getY();
        if (Double.isNaN(m)) m = 0.0000001;
        return new Vector2d((interpolator - b) / m, interpolator);
    }
    
    public static double averageOf(double a, double b) {
        return (a + b) / 2;
    }

    public static double crossOf(Vector2d a, Vector2d b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }

    public static double dotOf(Vector2d a, Vector2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static Vector2d swapXYOf(Vector2d vec) {
        return new Vector2d(vec.getY(), vec.getX());
    }

    /**
     * convert cartesian coordinates to polar coordinates
     * @param vec the (x,y) ordered pair in cartesian coordinates
     * @return ordered pair (r, theta) where theta is in radians
     */
    public static Vector2d toPolarCoordinates(Vector2d vec) {
        double r = Math.sqrt(vec.getX() * vec.getX() + vec.getY() * vec.getY());
        double theta = Math.atan2(vec.getY(), vec.getX());
        return new Vector2d(r, theta);
    }

    /**
     * convert polar coordinates to cartesian coordinates
     * @param vec the (r, theta) ordered pair in polar coordinates with theta in radians
     * @return ordered pair (x,y) in cartesian coordinates
     */
    public static Vector2d toCartesianCoordinates(Vector2d vec) {
        double x = vec.getX() * Math.cos(vec.getY());
        double y = vec.getX() * Math.sin(vec.getY());
        return new Vector2d(x,y);
    }

    public static double tanhErrorMap(double x) {
        double a = 1;
        double b = 1.1;
        return a * Math.tanh(b * x);
    }

    /**
     * snaps normal atan2 function to intervals of π/4.
     * @return atan2 output rounded to π/4
     */
    public static double peicewiseAtan2(double y, double x) {
        double theta = Math.atan2(y, x);

        if (theta >= -Math.PI / 8 && theta < Math.PI / 8) {
            return 0;
        }
        else if (theta >= Math.PI / 8 && theta < 3 * Math.PI / 8) {
            return Math.PI / 4;
        }
        else if (theta >= 3 * Math.PI / 8 && theta < 5 * Math.PI / 8) {
            return Math.PI / 2;
        }
        else if (theta >= 5 * Math.PI / 8 && theta < 7 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        }
        else if (theta >= -7 * Math.PI / 8 && theta < -5 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        }
        else if (theta >= -5 * Math.PI / 8 && theta < -3 * Math.PI / 8) {
            return -Math.PI / 2;
        }
        else if (theta >= -3 * Math.PI / 8 && theta < -Math.PI / 8) {
            return -Math.PI / 4;
        }
        return Math.PI;
    }

    /**
     * Although simple, this is the crown jewel of our vision code. Uses law of cosines to change distance between camera and sample to
     * distance between center of bot to sample, and angle difference from that of pointing straight forward.
     * @param distance Y distance to sample in inches
     * @return [angle from 0 to sample, distance from center to sample]
     */
    public static double[] crownJewel(double distance) {
        double xOffset = 4.5 + 5 / 16.0;
        double hypot = Math.hypot(distance, xOffset);

        //literally just law of cosines
        double theta = Math.acos((Math.pow(distance, 2) + Math.pow(hypot, 2) - Math.pow(xOffset, 2)) / (2 * distance * hypot));

        return new double[] {theta, hypot};
    }


}
