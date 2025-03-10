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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.CubicPath;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.DashOperations;
import org.firstinspires.ftc.teamcode.utility.PathList;

@Config
@Disabled
@TeleOp(name="Autotest", group="Linear Opmode")
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

        GVF gvf = new GVF(path, 0.7, 15, 0.7, telemetry);

        double[] x = new double[60];
        double[] y = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            Vector2d point = PathList.MidYellowToBlueBasket.getPoint(((double) i  / (x.length - 1)) * 2.9999);
            x[i] = point.getX();
            y[i] = point.getY();
        }

        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        swerve.setPosition(new Pose2d(-25, -48));

        while (opModeIsActive() && !isStopRequested()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            Pose2d pose = swerve.getPose();


            gvf.tuneValues(Kn, Kf, Ks);
            Vector2d out = null;
            try {
                out = gvf.output(new Vector2d(pose.getX(), pose.getY()));
            } catch (Exception e) {
                e.printStackTrace();
            }


            double rotation = gvf.headingOut(heading,swerve.getHeadingInDegrees());
            //swerve.drive(out.getX() * negX, out.getY() * negY, rotation);

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
            telemetry.addData("path8", PathList.BlueSideToScore.getControlPoint(7));
            telemetry.addData("path0", PathList.BlueSideToScore.getControlPoint(0));
            telemetry.addData("raw", PathList.BlueSideToScore.temp);
            telemetry.addData("outx",out.getX());
            telemetry.addData("outY", out.getY());
            telemetry.addData("gamex", gamepad1.left_stick_x);
            telemetry.addData("gamey", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
