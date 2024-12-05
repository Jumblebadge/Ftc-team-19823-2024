package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GVF {

    private CubicPath path;
    private Vector2d R, closestPoint, out;
    private Vector2d tangent, normal;
    private final FtcDashboard dashboard;
    private final PID headingPID = new PID(0.09,0.00188,0,0.025,1);
    private final PID xPID = new PID(0.6,0,0,0.275, 0.1);
    private final PID yPID = new PID(0.5,0,0,0.25, 0.1);
    private double Kn, Kf, Ks;
    private double distance, headingDistance;
    int count = 0;
    private final Telemetry telemetry;


    public Vector2d temp = new Vector2d(0,0), temp2 = new Vector2d(0,0);
    public double temp3 = 0;


    public GVF(FtcDashboard dashboard, CubicPath path, double Kn, double Kf, double Ks, Telemetry telemetry) {
        this.path = path;
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
        this.dashboard = dashboard;
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

    public double distanceFromEndPoint(Vector2d robot) { return robot.distTo(path.getControlPoint(7)); }

    public boolean isEnding() { return arcLengthRemaining() * 1.3 < Kf; }

    public boolean isDone(double positionTolerance, double headingTolerance) { return distance < positionTolerance && headingDistance < headingTolerance; }

    public Vector2d calculateGVF(Vector2d robot) {
        count++;
        calculateEverything(robot);
        temp = tangent;
        temp2 = normal;
        double error = calculateSinusoidalError();
        temp3 = error;
        out = tangent.minus(normal.times(Kn).times(error));
        telemetry.addData("error", error);
        double max = Math.max(Math.abs(out.getX()), Math.abs(out.getY()));
        if (max > 1) {
            out = new Vector2d(out.getX() / max, out.getY() / max);
        }
        out = out.times(Math.min(1,(distanceFromEndPoint(robot)) / Kf));
        telemetry.addData("errer",(path.getTotalArcLength() - path.arcLength) / Kf);
        out = new Vector2d(out.getX(), out.getY());
        return out.times(Ks);
    }

    public Vector2d calculatePID(Vector2d robot) {
        double yOut = yPID.pidOut(path.getControlPoint(7).getX(), robot.getY());
        double xOut = xPID.pidOut(path.getControlPoint(7).getX(), robot.getY());
        double max = Math.max(Math.abs(yOut), Math.abs(xOut));
        if (max > 1) {
            return new Vector2d(xOut / max, yOut / max);
        }
        return new Vector2d(-xOut, yOut);
    }

    public double headingOut(double heading, double targetHeading, boolean followTangent, boolean reversed) {
        double target = targetHeading;
        if (!isEnding() && followTangent) {
            if (reversed) target = AngleUnit.normalizeDegrees(180 + tangentHeading());
            else target = tangentHeading();
        }
        headingDistance = Math.abs(AngleUnit.normalizeDegrees(target - heading));
        return headingPID.pidAngleOut(target, heading);
    }

    public Vector2d output(Vector2d robot) {
        distance = Maths.distanceBetween(path.getControlPoint(7), robot);
        telemetry.addData("count",count);
        telemetry.addData("robot",robot);
        if (isEnding()) telemetry.addData("isending", arcLengthRemaining());
        else telemetry.addData("NOT ENDING","NOT");
        //drawPath(dashboard, path, new Pose2d(robot.getX(), robot.getY(), 0));
        telemetry.addData("tep",temp.toString());
        if (isEnding()) return calculatePID(robot);
        else return calculateGVF(robot);
    }

    public void drawPath(FtcDashboard dash, CubicPath path, Pose2d robot) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#51B53F");
        Vector2d[] points = new Vector2d[30];
        double[] x = new double[points.length];
        double[] y = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            points[i] = path.getPoint((double) i * 3 / (points.length - 1));
            x[i] = points[i].getX();
            y[i] = points[i].getY();
        }
        canvas.strokePolyline(x,y);
        //drawRobot(canvas, robot, out, path);
        drawPoint(canvas, new Pose2d(closestPoint.getX(), closestPoint.getY(), 0));
        drawVectors(canvas, new Pose2d(closestPoint.getX(), closestPoint.getY(), 0), out, path);
        dash.sendTelemetryPacket(packet);
    }

    public static void drawPoint(Canvas canvas, Pose2d pose) {
        canvas.fillCircle(pose.getX(), pose.getY(), 4);
    }

    public static void drawVectors(Canvas canvas, Pose2d pose, Vector2d out, CubicPath path) {
        Vector2d v = out.times(12);
        double x1 = pose.getX(), y1 = pose.getY();
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
        Vector2d tangent = path.getNormalizedTangent(path.guessT);
        Vector2d normal = path.getNormalizedNormal(path.guessT);
        canvas.setStroke("#B53F51");
        double x11 = path.getPoint(path.guessT).getX(), y11 = path.getPoint(path.guessT).getY();
        Vector2d v1 = tangent.times(6);
        double x12 = path.getPoint(path.guessT).getX() + v1.getX(), y12 = path.getPoint(path.guessT).getY() + v1.getY();
        canvas.strokeLine(x11, y11, x12, y12);
        double x21 = path.getPoint(path.guessT).getX(), y21 = path.getPoint(path.guessT).getY();
        Vector2d v2 = normal.times(6);
        double x22 = path.getPoint(path.guessT).getX() + v2.getX(), y22 = path.getPoint(path.guessT).getY() + v2.getY();
        canvas.strokeLine(x21, y21, x22, y22);
    }

}
