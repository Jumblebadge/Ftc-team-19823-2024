package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.maths.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.camera.CameraShenanigans;
import org.firstinspires.ftc.teamcode.utility.camera.YellowECircle;

import java.util.List;

@Config
@TeleOp(name="Gvsion", group="Linear Opmode")
public class Vision extends LinearOpMode {

    private double lastTheat = 0, lastY = 0;
    double headingTarget = 0;

    public static double r = 0;

    public static double lowY = 0, lowCr = 0, lowCb = 0, highY = 255, highCr = 255, highCb = 255;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        YellowECircle pipeline = new YellowECircle(telemetry);
        CameraShenanigans camera = new CameraShenanigans(hardwareMap, dashboard, pipeline);

        SlewRateLimiter limiter = new SlewRateLimiter();
        SlewRateLimiter limiter2 = new SlewRateLimiter();

        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        PID headingPID = new PID(0.2,0.017,0,0.1,5);
        ButtonDetector headingPIDtoggle  = new ButtonDetector();

        PivotingSlide slide = new PivotingSlide(hardwareMap);

        ElapsedTime hzTimer = new ElapsedTime();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        camera.setLightPower(1);

        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        camera.streamDash();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            previous1.copy(current1);
            current1.copy(gamepad1);
            previous2.copy(current2);
            current2.copy(gamepad2);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double heading = swerve.getJustHeadingInDegrees();

            if (pipeline.getSampleThetaValue() != 0) {
                lastTheat = pipeline.getSampleThetaValue() * (Math.abs(heading) / 10 + 1);
            }
            lastTheat = limiter.rateLimit(lastTheat, r);
            double h = lastTheat + headingTarget;


            double multiplier;
            if (gamepad1.left_trigger > 0.25) {
                multiplier = gamepad1.left_trigger;
            }
            else multiplier = 1;

            double rotation = headingPID.pidAngleOut(h, heading);

            if (current1.left_bumper && !previous1.left_bumper) {
                headingTarget = heading;
                headingPIDtoggle.toTrue();
            }

            if (gamepad1.options) {
                swerve.resetIMU();
            }

            if (current1.triangle && !previous1.triangle) {
                headingTarget = 0;
                headingPIDtoggle.toTrue();
            }
            if (current1.circle && !previous1.circle) {
                headingTarget = 90;
                headingPIDtoggle.toTrue();
            }
            if (current1.cross && !previous1.cross) {
                headingTarget = 180;
                headingPIDtoggle.toTrue();
            }
            if (current1.square && !previous1.square) {
                headingTarget = -90;
                headingPIDtoggle.toTrue();
            }
            if (Math.sqrt(Math.pow(gamepad1.right_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2)) > 0.175) {
                headingTarget = Math.toDegrees(Maths.peicewiseAtan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)) - 90;
            }

            //swerve.drive(-gamepad1.left_stick_x * multiplier, -gamepad1.left_stick_y * multiplier, rotation);

            pipeline.setThresh(lowY, lowCr, lowCb, highY, highCr, highCb);

            if (Math.abs(pipeline.getSampleThetaValue()) < 10 && pipeline.getSampleYValue() != -1) {
                lastY = pipeline.getSampleYValue() * 25.4 / 1.415;
            }
            //lastY = limiter2.rateLimit(lastY, r);
            if (lastY > 50) ;//slide.moveSlideTo(lastY - 50);
            else slide.toMin();



            //slide.update();

            telemetry.addData("hz", hzTimer.milliseconds());
            telemetry.addData("peice",Math.toDegrees(Maths.peicewiseAtan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)) - 90);
            telemetry.addData("theta", pipeline.getSampleThetaValue());
            telemetry.addData("Y", pipeline.getSampleYValue());
            telemetry.addData("headTTT", headingTarget);
            telemetry.addData("current ", heading);
            telemetry.addData("lastY", lastY);
            //telemetry.addData("pivot",slide.getPivotAngle());

            hzTimer.reset();
            telemetry.update();
        }
        camera.stopStreaming();
        pipeline.releaseMats();
        camera.setLightPower(0);
    }
}

