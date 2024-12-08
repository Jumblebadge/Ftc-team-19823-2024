package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DashOperations {

    public static void drawRobot(Canvas canvas, Pose2D pose2D) {
        Pose2d pose = new Pose2d(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
        canvas.strokeCircle(pose.getX(), pose.getY(), 3);
        Vector2d v = pose.headingVec().times(4);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose2D) {
        Pose2d pose = new Pose2d(pose2D.getX(), pose2D.getY(), pose2D.getHeading());
        canvas.strokeCircle(pose.getX(), pose.getY(), 3);
        Vector2d v = pose.headingVec().times(4);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawArrow(Canvas canvas, Vector2d start, Vector2d end) {
        canvas.strokeLine(start.getX(), start.getY(), end.getX(), end.getY());
        canvas.fillCircle(start.getX(), start.getY(), 1);
    }
}
