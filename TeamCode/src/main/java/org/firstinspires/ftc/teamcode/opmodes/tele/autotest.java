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

@Config
@TeleOp(name="autotest", group="Linear Opmode")
public class autotest extends LinearOpMode {

    public static double Kn = 0.7, Kf = 15, Ks = 0.75;
    private double heading = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);

        CubicPath path = new CubicPath(new double[] {
                -25,-48,37.7,-48.5,48,-60,48,-0.3,44.3,49,-12.7,48.8,-39.8,24.3,-47.4,-47.7
        });

        GVF gvf = new GVF(path, Kn, Kf, Ks, telemetry);

        PID headingPID = new PID(0.1,0.00188,0,0.05,1);

        double[] x = new double[60];
        double[] y = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            Vector2d point = path.getPoint(((double) i  / (x.length - 1)) * 2.9999);
            x[i] = point.getX();
            y[i] = point.getY();
        }

        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        swerve.setPosition(new Pose2d(-25, -48));

        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            Pose2d pose = swerve.getPose();


            gvf.tuneValues(Kn, Kf, Ks);
            Vector2d out = gvf.output(new Vector2d(pose.getX(), pose.getY()));


            double rotation = headingPID.pidAngleOut(heading, swerve.getHeadingInDegrees());
            swerve.drive(out.getX(), out.getY(), rotation);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            canvas.setStroke("#51B53F");
            canvas.strokePolyline(x,y);

            canvas.setStroke("#FF0000");
            DashOperations.drawRobot(canvas, pose);
            canvas.strokeLine(pose.getX(),pose.getY(),pose.getX() + out.getX() * 5,pose.getY() + out.getY() * 5);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("pose",pose.toString());
            telemetry.addData("gamepadx", gamepad1.left_stick_x);
            telemetry.addData("gamey",gamepad1.left_stick_y);
            telemetry.addData("outx",-out.getY());
            telemetry.addData("outy",-out.getX());
            telemetry.addData("milis",hztimer.milliseconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
