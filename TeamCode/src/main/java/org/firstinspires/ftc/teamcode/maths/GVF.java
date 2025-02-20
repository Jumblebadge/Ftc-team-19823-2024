package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GVF {

    private CubicPath path;
    private Vector2d R, closestPoint, out;
    private Vector2d tangent, normal;
    ElapsedTime time = new ElapsedTime();
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
        time.reset();
        //calculateGVF(path.getPoint(0));
    }

    public void setPath(CubicPath path, double Kn, double Kf, double Ks, Pose2d robot) {
        this.path = path;
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
        calculateEverything(new Vector2d(robot.getX(), robot.getY()));
        time.reset();
    }

    public CubicPath getPath() {
        return path;
    }

    public void tuneValues(double Kn, double Kf, double Ks) {
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
    }

    public void setKs(double Ks) {
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

    public void calculateEverything(Vector2d robot) {
        poseError = distanceFromEndPoint(robot);
        closestPoint = path.findClosestPointOnPath(robot);
        tangent = path.getNormalizedTangent(path.guessT);
        normal = path.getNormalizedNormal(path.guessT);
        R = robot.minus(closestPoint);
        telemetry.addData("closestPoint",closestPoint);
    }

    public double tangentHeading() { return AngleUnit.normalizeDegrees(90 + (360 - (Maths.angleOf(tangent) * (180 / Math.PI)))); }

    public double arcLengthRemaining() { return path.getTotalArcLength() - path.arcLength; }

    public double distanceFromEndPoint(Vector2d robot) { return robot.distTo(path.getPoint(2.9999)); }

    public boolean isEnding() { return arcLengthRemaining() * 1.3 < Kf; }

    public boolean isDone(double positionTolerance, double headingTolerance) { return poseError < positionTolerance && headingError < headingTolerance; }

    public boolean isDone(Vector2d robot) { return distanceFromEndPoint(robot) < Kf; }

    public Vector2d calculateGVF(Vector2d robot) {
        count++;
        double error = calculateSinusoidalError();
        out = tangent.minus(normal.times(Kn).times(error));
        telemetry.addData("sin error: ", error);
        double max = Math.max(Math.abs(out.getX()), Math.abs(out.getY()));
        if (max > 1) out = out.div(max);
        out = out.times(Math.min(1,(distanceFromEndPoint(robot)) / Kf));
        out = new Vector2d(out.getY(), out.getX());
        return out.times(Ks);
    }

    public Vector2d calculatePID(Vector2d robot) {
        return calculatePID(path.getPoint(2.9999), robot);
    }

    public Vector2d calculatePID(Vector2d target, Vector2d robot) {
        double xOut = xPID.pidOut(target.getX(), robot.getX());
        double yOut = yPID.pidOut(target.getY(), robot.getY());
        double max = Math.max(Math.abs(yOut), Math.abs(xOut));
        if (max > 1) {
            xOut /= max;
            yOut /= max;
        }
        return new Vector2d(yOut, xOut);
    }

    public double headingOut(double targetHeading, double currentHeading) {
        headingError = Math.abs(AngleUnit.normalizeDegrees(targetHeading - currentHeading));
        return headingPID.pidAngleOut(targetHeading, currentHeading);
    }

    public double headingOut(double currentHeading, double endTangent, boolean reversed) {
        double target = endTangent;
        if (!isEnding()) {
            if (reversed) target = AngleUnit.normalizeDegrees(180 + tangentHeading());
            else target = tangentHeading();
        }
        headingError = Math.abs(AngleUnit.normalizeDegrees(target - currentHeading));
        return headingPID.pidAngleOut(target, currentHeading);
    }


    public Vector2d output(Vector2d robot) {
        Vector2d out;
        double magnitude;
        calculateEverything(robot);
        temp3 = path.getTotalArcLength();
        temp2 = path.arcLength;
        telemetry.addData("robot",robot);
        telemetry.addData("PATH ENDING?", isEnding());
        telemetry.addData("arc length remaining: ",arcLengthRemaining());
        telemetry.addData("tep",temp.toString());
        magnitude = Range.clip(time.seconds() * 3, 0, 1);
        if (isEnding()) out = calculatePID(robot);
        else if (Maths.epsilonEquals(path.guessT, 0)) out = calculatePID(path.getPoint(path.distanceToT(7.5)), robot);
        else out = calculateGVF(robot);
        return out.times(magnitude);
    }

}
