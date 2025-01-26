package org.firstinspires.ftc.teamcode.opmodes.auto;

//Import EVERYTHING we need

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.ThreeAxisClaw;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.DashOperations;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;
import org.firstinspires.ftc.teamcode.utility.PathList;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name="Blue Park", group="Linear Opmode")
public class BluePark extends LinearOpMode {

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);

        GVF gvf = new GVF(PathList.BluePark, 0.5, 5, 0.7, telemetry);

        PivotingSlide slide = new PivotingSlide(hardwareMap, true);
        ButtonDetector pivotToggle = new ButtonDetector();

        ThreeAxisClaw claw = new ThreeAxisClaw(hardwareMap);
        ButtonDetector clawToggle = new ButtonDetector(true);
        claw.setClawClose();
        claw.setRotatorTo0();
        ButtonDetector wristToggle = new ButtonDetector(true);

        //Bulk sensor reads
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        waitForStart();

        swerve.setPosition(new Pose2d(-16.5, 65));

        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            Pose2d pose = swerve.getPose();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            DashOperations.drawRobot(canvas, pose);
            dashboard.sendTelemetryPacket(packet);

            Vector2d out = gvf.output(new Vector2d(pose.getX(), pose.getY()));

            double rotation = gvf.headingOut(-90, swerve.getHeadingInDegrees());
            swerve.drive(out.getX(), out.getY(), rotation);


            slide.toMin();
            clawToggle.toFalse();
            wristToggle.toFalse();
            pivotToggle.toFalse();

            if (clawToggle.isTrue()) {
                claw.setClawClose();
            }
            else claw.setClawOpen();

            if (wristToggle.isTrue()) {
                claw.setWristDown();
            }
            else claw.setWristUp();

            if (pivotToggle.isTrue()) {
                slide.movePivotTo(85);
            }
            else if (slide.getSlidePosition() < 50) {
                slide.movePivotTo(0);
            }

            slide.update();

            telemetry.addData("pose",pose.toString());
            telemetry.addData("total arc", gvf.temp3);
            telemetry.addData("poseError",gvf.poseError);
            telemetry.addData("headingError", gvf.headingError);
            telemetry.addData("path minus",PathList.BlueSideToScore.temp);
            telemetry.addData("path arc in gvf",PathList.BlueSideToScore.temp2);
            telemetry.addData("arc",gvf.arcLengthRemaining());
            telemetry.addData("path", Arrays.toString(gvf.getPath().getControlPointList()));
            telemetry.update();
        }
    }
}
