package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.maths.CubicPath;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.DashOperations;
import org.firstinspires.ftc.teamcode.utility.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utility.PathList;

import java.util.Arrays;

@Config
@TeleOp(name="autotest", group="Linear Opmode")
public class autotest extends LinearOpMode {

    public static double Kn = 0.7, Kf = 15, Ks = 0.75, negX = 1, negY = 1;
    private double heading = 0, endingCount = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);

        CubicPath path = new CubicPath(new double[] {
                -24.3,-48,-0.8,-46,37,-60.5,47.6,-45.7,53,-13.6,50.5,-2.6,47.7,11.5,47.2,48.8
        });

        GVF gvf = new GVF(PathList.BlueSideToScore, 0.7, 15, 0.7, telemetry);

        double[] x = new double[60];
        double[] y = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            Vector2d point = PathList.BlueSideToScore.getPoint(((double) i  / (x.length - 1)) * 2.9999);
            x[i] = point.getX();
            y[i] = point.getY();
        }

        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        swerve.setPosition(new Pose2d(36, 60, -90));

        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            Pose2d pose = swerve.getPose();


            gvf.tuneValues(Kn, Kf, Ks);
            Vector2d out = gvf.output(new Vector2d(pose.getX(), pose.getY()));


            double rotation = gvf.headingOut(heading,swerve.getHeadingInDegrees(), false, false);
            //swerve.drive(out.getX() * negX, out.getY() * negY, 0);

            if (gvf.isEnding()) endingCount ++;

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            canvas.setStroke("#51B53F");
            canvas.strokePolyline(x,y);

            canvas.setStroke("#FF0000");
            DashOperations.drawRobot(canvas, pose);
            canvas.strokeLine(pose.getX(),pose.getY(),pose.getX() + out.getX() * 5,pose.getY() + out.getY() * 5);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("pose",pose.toString());
            telemetry.addData("poseError",gvf.poseError);
            telemetry.addData("headingError", gvf.headingError);
            telemetry.addData("path", Arrays.toString(PathList.LeftYellowToBlueBasket.getControlPointList()));
            telemetry.addData("path8", PathList.BlueSideToScore.getControlPoint(7));
            telemetry.addData("path0", PathList.BlueSideToScore.getControlPoint(0));
            telemetry.addData("raw", Arrays.toString(PathList.BlueSideToScore.temp));
            telemetry.update();
        }
    }
}