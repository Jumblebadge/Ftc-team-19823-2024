package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GVF {

    CubicPath path;
    Vector2d R, closestPoint, out, tangent;
    FtcDashboard dashboard;
    PID headingPID = new PID(0.1,0.001,0,0.75,0.1);
    PID xPID = new PID(0.6,0,0,0.275, 0.1);
    PID yPID = new PID(0.5,0,0,0.25, 0.1);
    double Kn, Kf, Ks;
    private double distance, headingDistance;
    Vector2d temp = new Vector2d(0,0);
    int count = 0;
    Telemetry telemetry;


    public GVF(FtcDashboard dashboard, CubicPath path, double Kn, double Kf, double Ks, Telemetry telemetry) {
        this.path = path;
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
        this.dashboard = dashboard;
        this.telemetry = telemetry;
        calculateGVF(this.path.getControlPoint(0));
    }

    public void setPath(CubicPath path, double Kn, double Kf, double Ks) {
        this.path = path;
        this.Kn = Kn;
        this.Kf = Kf;
        this.Ks = Ks;
        calculateGVF(this.path.getControlPoint(0));
    }

    public double calculateError(Vector2d tangent) {
        double magnitudeOfR = Maths.magnitudeOf(R);
        magnitudeOfR = Math.pow(1.3, magnitudeOfR - 10) - 0.073;
        magnitudeOfR *= -Math.signum(Maths.crossOf(R,tangent));
        if (Math.abs(magnitudeOfR) > 100) return Maths.magnitudeOf(R) * -(Math.signum(Maths.crossOf(R,tangent)));
        return magnitudeOfR;
    }

    public void calculateEverything(Vector2d Robot) {
        closestPoint = path.findClosestPointOnPath(Robot);
        telemetry.addData("closestPoind",closestPoint);
        R = new Vector2d(Robot.x - closestPoint.x, Robot.y - closestPoint.y);
    }

    public double tangentHeading() { return AngleUnit.normalizeDegrees(90 + (360 - (Maths.angleOf(tangent) * (180 / Math.PI)))); }

    public double distanceFromEnd() { return path.getTotalArcLength() - path.arcLength; }

    public boolean isEnding() { return distanceFromEnd() * 1.3 < Kf; }

    public boolean isDone(double positionTolerance, double headingTolerance) { return distance < positionTolerance && headingDistance < headingTolerance; }

    public Vector2d calculateGVF(Vector2d Robot) {
        count++;
        temp = Robot;
        calculateEverything(Robot);
        tangent = path.getNormalizedTangent(path.guessT);
        Vector2d normal = path.getNormalizedNormal(path.guessT);
        double error = calculateError(tangent);
        out = tangent.minus(normal.times(Kn).times(error));
        telemetry.addData("error", error);
        double max = Math.max(Math.abs(out.x), Math.abs(out.y));
        if (max > 1) {
            out = new Vector2d(out.x / max, out.y / max);
        }
        out = out.times(Math.min(1,(distanceFromEnd()) / Kf));
        telemetry.addData("errer",(path.getTotalArcLength() - path.arcLength) / Kf);
        out = new Vector2d(-out.x, out.y);
        return out.times(Ks);
    }

    public Vector2d calculatePID(Vector2d robot) {
        double yOut = yPID.pidOut(path.getControlPoint(7).x - robot.y);
        double xOut = xPID.pidOut(path.getControlPoint(7).x - robot.y);
        double max = Math.max(Math.abs(yOut), Math.abs(xOut));
        if (max > 1) {
            return new Vector2d(xOut / max, yOut / max);
        }
        return new Vector2d(-xOut, yOut);
    }

    public Vector2d cameraPID(Vector2d xy) {
        double yOut = yPID.pidOut(13 - xy.y);
        double xOut = xPID.pidOut(-xy.x);
        double max = Math.max(Math.abs(yOut), Math.abs(xOut));
        if (max > 1) {
            return new Vector2d(xOut / max, yOut / max);
        }
        return new Vector2d(yOut, xOut);
    }

    public boolean isCameraDone(Vector2d xy) {
        return 13 - xy.y < 3 && -xy.x < 3;
    }

    public double headingOut(double heading, double targetHeading, boolean followTangent, boolean reversed) {
        double target = targetHeading;
        if (!isEnding() && followTangent) {
            if (reversed) target = AngleUnit.normalizeDegrees(180 + tangentHeading());
            else target = tangentHeading();
        }
        headingDistance = Math.abs(AngleUnit.normalizeDegrees(target - heading));
        return headingPID.pidOut(AngleUnit.normalizeDegrees(target - heading));
    }

    public Vector2d output(Vector2d robot) {
        distance = Maths.distanceBetween(path.getControlPoint(7), robot);
        telemetry.addData("count",count);
        telemetry.addData("robot",robot);
        if (isEnding()) telemetry.addData("isending",distanceFromEnd());
        drawPath(dashboard, path, new Pose2d(robot.x, robot.y, 0));
        telemetry.addData("tep",temp);
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
            points[i] = path.getPoint((double) i * 3 / points.length);
            x[i] = points[i].x;
            y[i] = points[i].y;
        }
        canvas.strokePolyline(x,y);
        //drawRobot(canvas, robot, out, path);
        drawPoint(canvas, new Pose2d(closestPoint.x, closestPoint.y, 0));
        drawVectors(canvas, new Pose2d(closestPoint.x, closestPoint.y, 0), out, path);
        dash.sendTelemetryPacket(packet);
    }

    public static void drawPoint(Canvas canvas, Pose2d pose) {
        canvas.fillCircle(pose.position.x, pose.position.y, 4);
    }

    public static void drawVectors(Canvas canvas, Pose2d pose, Vector2d out, CubicPath path) {
        Vector2d v = out.times(12);
        double x1 = pose.position.x, y1 = pose.position.y;
        double x2 = pose.position.x + v.x, y2 = pose.position.y + v.y;
        canvas.strokeLine(x1, y1, x2, y2);
        Vector2d tangent = path.getNormalizedTangent(path.guessT);
        Vector2d normal = path.getNormalizedNormal(path.guessT);
        canvas.setStroke("#B53F51");
        double x11 = path.getPoint(path.guessT).x, y11 = path.getPoint(path.guessT).y;
        Vector2d v1 = tangent.times(6);
        double x12 = path.getPoint(path.guessT).x + v1.x, y12 = path.getPoint(path.guessT).y + v1.y;
        canvas.strokeLine(x11, y11, x12, y12);
        double x21 = path.getPoint(path.guessT).x, y21 = path.getPoint(path.guessT).y;
        Vector2d v2 = normal.times(6);
        double x22 = path.getPoint(path.guessT).x + v2.x, y22 = path.getPoint(path.guessT).y + v2.y;
        canvas.strokeLine(x21, y21, x22, y22);
    }

}
