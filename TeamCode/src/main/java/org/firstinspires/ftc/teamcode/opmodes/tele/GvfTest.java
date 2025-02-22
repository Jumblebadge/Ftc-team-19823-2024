package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utility.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.maths.CubicPath;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.utility.GoBildaPinpointDriver;

import java.util.Locale;

@Config
@TeleOp(name="gvf test", group="Linear Opmode")
public class GvfTest extends LinearOpMode {

    public static double AX = -40, AY = 0, BX = -40, BY = 30, CX = -10, CY = 30, DX = -10, DY = 0, FX = 10, FY = -30, GX = 10, GY = 0, IX = 40, IY = 30, JX = 40, JY = 0;
    public static double distance = 0, Kn = 1, Kf = 1, Ks = 1, negX = 1, negY = 1;
    private double average, count;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.resetPosAndIMU();

        CubicPath path = new CubicPath(new double[] {AX, AY, BX, BY, CX, CY, DX, DY, FX, FY, GX, GY, IX, IY, JX, JY});

        GVF gvf = new GVF(path, Kn, Kf, Ks, telemetry);

        Vector2d[] points = new Vector2d[60];

        //class to swerve the swerve
        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {

            path.setControlPointCoordinates(new double[] {AX, AY, BX, BY, CX, CY, DX, DY, FX, FY, GX, GY, IX, IY, JX, JY});

            gvf.setPath(path, Kn, Kf, Ks, new Pose2d(0,0,0));

            double[] x = new double[points.length];
            double[] y = new double[points.length];
            for (int i = 0; i < points.length; i++) {
                points[i] = path.getPoint(((double) i  / (points.length - 1)) * 2.9999);
                x[i] = points[i].getX();
                y[i] = points[i].getY();
            }
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            canvas.setStrokeWidth(1);
            canvas.setFill("#FFC0CB");
            canvas.fillCircle(path.getControlPoint(0).getX(),path.getControlPoint(0).getY(), 1);
            canvas.fillCircle(path.getControlPoint(1).getX(),path.getControlPoint(1).getY(), 1);
            canvas.fillCircle(path.getControlPoint(2).getX(),path.getControlPoint(2).getY(), 1);
            canvas.fillCircle(path.getControlPoint(3).getX(),path.getControlPoint(3).getY(), 1);
            canvas.fillCircle(path.getControlPoint(4).getX(),path.getControlPoint(4).getY(), 1);
            canvas.fillCircle(path.getControlPoint(5).getX(),path.getControlPoint(5).getY(), 1);
            canvas.fillCircle(path.getControlPoint(6).getX(),path.getControlPoint(6).getY(), 1);
            canvas.fillCircle(path.getControlPoint(7).getX(),path.getControlPoint(7).getY(), 1);

            Vector2d robot = new Vector2d(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));
            Vector2d out = gvf.calculateGVF(robot);
            out = new Vector2d(out.getX() * negX, out.getY() * negY);
            telemetry.addData("out",out.toString());
            Vector2d closestPont = path.getPoint(path.guessT);
            canvas.fillCircle(closestPont.getX(), closestPont.getY(), 2);
            drawRobot(canvas, pos);

            canvas.setStroke("#FF0000");
            //drawArrow(canvas, new Vector2d(0,0), gvf.temp.times(5));
            canvas.strokeLine(robot.getX(),robot.getY(),robot.getX() + out.getX() * 5,robot.getY() + out.getY() * 5);
            //canvas.strokeLine(0,0, gvf.temp.getX() * 5, gvf.temp.getY() * 5);
            //canvas.strokeLine(0,0, gvf.temp2.getX() * 5, gvf.temp2.getY() * 5);
            //canvas.strokeLine(0,0, (gvf.temp.getX() - gvf.temp2.getX()  * gvf.temp3) * 5, (gvf.temp.getY() - gvf.temp2.getY() * gvf.temp3) * 5);
            //canvas.strokeLine(robot.getX(),robot.getY(),robot.getX() + gvf.temp2.getX() * 5  * gvf.temp3,robot.getY() + gvf.temp2.getY() * 5 * gvf.temp3);
            //canvas.strokeLine(closestPont.getX() + gvf.temp2.getX() * 5, closestPont.getY() + gvf.temp2.getY() * 5, closestPont.getX(), closestPont.getY());
            //canvas.strokeLine(robot.getX() + (gvf.temp.getX() - gvf.temp2.getX() * gvf.temp3) * -5 * negX, robot.getY() + (gvf.temp.getY() - gvf.temp2.getY() * gvf.temp3) * 5 * negY, robot.getX(), robot.getY());
            //canvas.strokeLine(closestPont.getX() + gvf.temp.getX() * 5, closestPont.getY() + gvf.temp.getY() * 5, closestPont.getX(), closestPont.getY());


            canvas.setStroke("00FFFF");
            for (int i = -10; i < 11; i++) {
                for (int j = -10; j < 11; j++) {
                    Vector2d vec = new Vector2d(i * 7, j * 7);
                    out = gvf.calculateGVF(vec);
                    drawArrow(canvas, vec, vec.plus(out.times(5)));
                }
            }

            canvas.setStroke("#51B53F");
            canvas.strokePolyline(x,y);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("arclength", path.getTotalArcLength());
            telemetry.addData("distance",distance);
            telemetry.addData("T",path.distanceToT(distance));
            telemetry.addData("robot",new Vector2d(pos.getX(DistanceUnit.INCH) - closestPont.getX(), pos.getY(DistanceUnit.INCH) - closestPont.getY()));
            telemetry.addData("closest",closestPont.toString());

            telemetry.addData("millis", hztimer.seconds());
            average += hztimer.milliseconds();
            count++;
            telemetry.addData("avearag",average / count);


            hztimer.reset();
            telemetry.update();
        }


    }

    public static void drawRobot(Canvas canvas, Pose2D pose1) {
        Pose2d pose = new Pose2d(pose1.getX(DistanceUnit.INCH), pose1.getY(DistanceUnit.INCH), pose1.getHeading(AngleUnit.RADIANS));
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
