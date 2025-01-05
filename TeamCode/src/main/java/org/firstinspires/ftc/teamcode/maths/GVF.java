package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GVF {

    private CubicPath path;
    private Vector2d R, closestPoint, out;
    private Vector2d tangent, normal;
    PID headingPID = new PID(0.1,0.00188,0,0.05,1);
    private final PID xPID = new PID(0.75,0.001,0.5,0.5, 1);
    private final PID yPID = new PID(0.75,0.001,0.5,0.5, 1);
    private double Kn, Kf, Ks;
    public double poseError, headingError;
    int count = 0;
    private final Telemetry telemetry;


    public Vector2d temp = new Vector2d(0,0);
    public double temp3 = 0, temp2 = 0;

    //Some recommended values
    //Kn0.7, Kf15, Ks0.75
    public GVF(CubicPath path, double Kn, double Kf, double Ks, Telemetry telemetry) {
        this.path = path;
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
        this.telemetry = telemetry;
        calculateGVF(path.getControlPoint(0));
    }

    public void setPath(CubicPath path, double Kn, double Kf, double Ks) {
        this.path = path;
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
        calculateGVF(path.getControlPoint(0));
    }

    public CubicPath getPath() {
        return path;
    }

    public void tuneValues(double Kn, double Kf, double Ks) {
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
    }

    public double calculateExponentialError() {
        double magnitudeOfR = Maths.magnitudeOf(R);
        magnitudeOfR = Math.pow(1.3, magnitudeOfR - 10) - 0.073;
        magnitudeOfR *= -Math.signum(Maths.crossOf(R,tangent));
        if (Math.abs(magnitudeOfR) > 100) return Maths.magnitudeOf(R) * -(Math.signum(Maths.crossOf(R,tangent)));
        return magnitudeOfR;
    }

    public double calculateSinusoidalError() {
        double horizontalStretch = 4.6;
        double power = 2.6;
        double verticalStretch = 5;
        double magnitudeOfR = Maths.magnitudeOf(R);
        double sign = -Math.signum(Maths.crossOf(R, tangent));
        double internal = Math.pow(Math.abs(magnitudeOfR / horizontalStretch), power);
        return verticalStretch * (internal * sign) / (1 + internal);
    }

    public void calculateEverything(Vector2d Robot) {
        closestPoint = path.findClosestPointOnPath(Robot);
        tangent = path.getNormalizedTangent(path.guessT);
        normal = path.getNormalizedNormal(path.guessT);
        R = Robot.minus(closestPoint);
        telemetry.addData("closestPoint",closestPoint);
    }

    public double tangentHeading() { return AngleUnit.normalizeDegrees(90 + (360 - (Maths.angleOf(tangent) * (180 / Math.PI)))); }

    public double arcLengthRemaining() { return path.getTotalArcLength() - path.arcLength; }

    public double distanceFromEndPoint(Vector2d robot) { return robot.distTo(path.getPoint(2.9999)); }

    public boolean isEnding() { return arcLengthRemaining() * 1.3 < Kf; }

    public boolean isDone(double positionTolerance, double headingTolerance) { return poseError < positionTolerance && headingError < headingTolerance; }

    public Vector2d calculateGVF(Vector2d robot) {
        count++;
        calculateEverything(robot);
        double error = calculateSinusoidalError();
        out = tangent.minus(normal.times(Kn).times(error));
        telemetry.addData("error", error);
        double max = Math.max(Math.abs(out.getX()), Math.abs(out.getY()));
        if (max > 1) out = out.div(max);
        out = out.times(Math.min(1,(distanceFromEndPoint(robot)) / Kf));
        telemetry.addData("errer",(path.getTotalArcLength() - path.arcLength) / Kf);
        out = new Vector2d(out.getX(), -out.getY());
        return out.times(Ks);
    }

    public Vector2d calculatePID(Vector2d robot) {
        double xOut = xPID.pidOut(path.getPoint(2.9999).getX(), robot.getX());
        double yOut = yPID.pidOut(path.getPoint(2.9999).getY(), robot.getY());
        double max = Math.max(Math.abs(yOut), Math.abs(xOut));
        if (max > 1) {
            xOut /= max;
            yOut /= max;
        }
        return new Vector2d(xOut, -yOut);
    }

    public double headingOut(double targetHeading, double currentHeading, boolean followTangent, boolean reversed) {
        double target = targetHeading;
        if (!isEnding() && followTangent) {
            if (reversed) target = AngleUnit.normalizeDegrees(180 + tangentHeading());
            else target = tangentHeading();
        }
        headingError = Math.abs(AngleUnit.normalizeDegrees(target - currentHeading));
        return headingPID.pidAngleOut(target, currentHeading);
    }

    public Vector2d output(Vector2d robot) {
        poseError = Maths.distanceBetween(path.getPoint(2.9999), robot);
        temp3 = path.getTotalArcLength();
        temp2 = path.arcLength;
        telemetry.addData("count",count);
        telemetry.addData("robot",robot);
        if (isEnding()) telemetry.addData("IS ENDING", arcLengthRemaining());
        else telemetry.addData("NOT ENDING",arcLengthRemaining());
        //drawPath(dashboard, path, new Pose2d(robot.getX(), robot.getY(), 0));
        telemetry.addData("tep",temp.toString());
        if (isEnding()) return calculatePID(robot);
        else return calculateGVF(robot);
    }

}
