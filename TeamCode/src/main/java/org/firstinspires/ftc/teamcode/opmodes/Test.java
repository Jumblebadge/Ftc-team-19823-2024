package org.firstinspires.ftc.teamcode.opmodes;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.Bezier;
import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.MedianFilter;
import org.firstinspires.ftc.teamcode.subsystems.ThreeAxisClaw;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

@Config
@TeleOp(name="tst", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static double AX = -10, AY = 0, BX = -10, BY = 13, CX = 10, CY = 13, DX = 10, DY = 0;
    public static double distance = 0;
    private double average, count;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        Bezier bezier = new Bezier(new Vector2d(AX, AY), new Vector2d(BX, BY), new Vector2d(CX, CY), new Vector2d(DX, DY));

        //class to swerve the swerve
        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {


            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            //bezier.setControlPoints(new Vector2d(AX, AY), new Vector2d(BX, BY), new Vector2d(CX, CY), new Vector2d(DX, DY));

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            canvas.setStroke("#51B53F");
            Vector2d[] points = new Vector2d[51];
            double[] x = new double[points.length];
            double[] y = new double[points.length];
            for (int i = 0; i < points.length; i++) {
                points[i] = bezier.getPoint((double) i / (points.length - 1));
                x[i] = points[i].getX();
                y[i] = points[i].getY();
            }
            canvas.strokePolyline(x,y);
            canvas.setFill("#FFC0CB");
            canvas.fillCircle(bezier.getA().getX(),bezier.getA().getY(), 1);
            canvas.fillCircle(bezier.getB().getX(),bezier.getB().getY(), 1);
            canvas.fillCircle(bezier.getC().getX(),bezier.getC().getY(), 1);
            canvas.fillCircle(bezier.getD().getX(),bezier.getD().getY(), 1);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("arclength", bezier.getTotalArcLength());
            telemetry.addData("lookup", bezier.lookup[bezier.accuracy].getY());
            telemetry.addData("distance",distance);
            telemetry.addData("T",bezier.distanceToT(distance));

            telemetry.addData("millis", hztimer.seconds());
            average += hztimer.milliseconds();
            count++;
            telemetry.addData("avearag",average / count);


            hztimer.reset();
            telemetry.update();
        }
    }
}
