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
import org.firstinspires.ftc.teamcode.maths.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.DashOperations;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;
import org.firstinspires.ftc.teamcode.utility.PathList;
import org.firstinspires.ftc.teamcode.utility.camera.BrushColor;
import org.firstinspires.ftc.teamcode.utility.camera.CameraShenanigans;
import org.firstinspires.ftc.teamcode.utility.camera.YellowECircle;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name="Close 6 Sample", group="Linear Opmode")
public class Close6Sample extends LinearOpMode {

    private double headingTarget = 0;
    private int cycleCount = 0;
    private int taskNumber = 0, lastTaskNumber = 0;
    private double lastTheta = 0, lastY = 0;
    boolean pivotClear = false;
    boolean wristToggle = true;
    ElapsedTimeW timer = new ElapsedTimeW();

    private enum ApexStates {
        PRELOAD,
        CYCLE,
        CAMERA,
        STANDBY
    }

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);

        GVF gvf = new GVF(PathList.SideToScore, 0.5, 5, 0.8, telemetry);

        YellowECircle pipeline = new YellowECircle(telemetry);
        CameraShenanigans camera = new CameraShenanigans(hardwareMap, dashboard, pipeline, false);

        SlewRateLimiter limiter = new SlewRateLimiter();

        PivotingSlide slide = new PivotingSlide(hardwareMap);
        ButtonDetector pivotToggle = new ButtonDetector();

        Intake intake = new Intake(hardwareMap);
        ButtonDetector latchToggle = new ButtonDetector(true);
        ButtonDetector rotatorToggle = new ButtonDetector();

        intake.setLatchClose();
        intake.setSpinIn();
        intake.setWristUp();

        ApexStates state = ApexStates.PRELOAD;

        //Bulk sensor reads
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        camera.setLightPower(1);

        waitForStart();

        intake.setWristPosition(0.12);

        swerve.setPosition(new Pose2d(31.5, 65));

        while (opModeIsActive() && !isStopRequested()) {

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            Pose2d pose = swerve.getPose();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            DashOperations.drawRobot(canvas, pose);
            dashboard.sendTelemetryPacket(packet);

            Vector2d out = gvf.output(new Vector2d(pose.getX(), pose.getY()));

            double rotation = gvf.headingOut(headingTarget + lastTheta ,swerve.getHeadingInDegrees());
            swerve.drive(out.getX(), out.getY(), rotation);

            switch(state) {
                case PRELOAD:
                    if (taskNumber == 0) {
                        pivotToggle.toTrue();
                        wristToggle = true;
                        //intake.setSpinIn();
                        headingTarget = -135;
                        taskNumber++;
                    }
                    else if (taskNumber == 1 && gvf.isDone(4, 7)) {
                        slide.toMax();
                        taskNumber++;
                    }
                    else if (taskNumber == 2 && slide.isTimeDone()) {
                        rotatorToggle.toTrue();
                        //intake.setSpin0();
                        taskNumber++;
                    }
                    else if (taskNumber == 3 && timer.seconds() > 0.5) {
                        latchToggle.toFalse();
                        //intake.setSpinIn();
                        taskNumber++;
                    }
                    else if (taskNumber == 4 && timer.seconds() > 0.4) {
                        pivotClear = true;
                        //intake.setSpin0();
                        rotatorToggle.toFalse();
                        taskNumber++;
                    }
                    else if (taskNumber == 5 && timer.seconds() > 0.3) {
                        slide.toMin();
                        pivotToggle.toFalse();
                        taskNumber++;
                    }
                    else if (taskNumber == 6 && slide.isTimeDone() && timer.seconds() > 0.2) {
                        taskNumber = 0;
                        headingTarget = -100;
                        state = ApexStates.CYCLE;
                        wristToggle = true;
                        latchToggle.toTrue();
                    }
                    break;
                case CYCLE:
                    if (cycleCount == 0) {
                        if (taskNumber == 0 && slide.isTimeDone() && gvf.isDone(4, 7)) {
                            slide.toSetPoint3();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && slide.isTimeDone()) {
                            slide.toMin();
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && slide.isTimeDone()) {
                            pivotToggle.toTrue();
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && gvf.isDone(4, 7)) {
                            slide.toMax();
                            //intake.setSpin0();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && slide.isTimeDone()) {
                            rotatorToggle.toTrue();
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.5) {
                            latchToggle.toFalse();
                            //intake.setSpinIn();
                            taskNumber++;
                        }
                        else if (taskNumber == 6 && timer.seconds() > 0.4) {
                            pivotClear = true;
                            //intake.setSpin0();
                            rotatorToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 7 && timer.seconds() > 0.3) {
                            slide.toMin();
                            pivotToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 8 && slide.isTimeDone() && timer.seconds() > 0.2) {
                            taskNumber = 0;
                            headingTarget = -85;
                            cycleCount = 1;
                            wristToggle = true;
                            latchToggle.toTrue();
                        }
                    }
                    else if (cycleCount == 1) {
                        if (taskNumber == 0 && slide.isTimeDone() && gvf.isDone(4, 7)) {
                            slide.toSetPoint3();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && slide.isTimeDone()) {
                            slide.toMin();
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && slide.isTimeDone()) {
                            pivotToggle.toTrue();
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && gvf.isDone(4, 7)) {
                            slide.toMax();
                            //intake.setSpin0();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && slide.isTimeDone()) {
                            rotatorToggle.toTrue();
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.5) {
                            latchToggle.toFalse();
                            //intake.setSpinIn();
                            taskNumber++;
                        }
                        else if (taskNumber == 6 && timer.seconds() > 0.4) {
                            pivotClear = true;
                            //intake.setSpin0();
                            rotatorToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 7 && timer.seconds() > 0.3) {
                            slide.toMin();
                            pivotToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 8 && slide.isTimeDone() && timer.seconds() > 0.2) {
                            taskNumber = 0;
                            headingTarget = -70;
                            cycleCount = 2;
                            wristToggle = true;
                            latchToggle.toTrue();
                        }
                    }
                    else if (cycleCount == 2) {
                        if (taskNumber == 0 && slide.isTimeDone() && gvf.isDone(4, 7)) {
                            slide.toSetPoint3();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && slide.isTimeDone()) {
                            slide.toMin();
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && slide.isTimeDone()) {
                            pivotToggle.toTrue();
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && gvf.isDone(4, 7)) {
                            slide.toMax();
                            //intake.setSpin0();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && slide.isTimeDone()) {
                            rotatorToggle.toTrue();
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.5) {
                            latchToggle.toFalse();
                            //intake.setSpinIn();
                            taskNumber++;
                        }
                        else if (taskNumber == 6 && timer.seconds() > 0.4) {
                            pivotClear = true;
                            //intake.setSpin0();
                            rotatorToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 7 && timer.seconds() > 0.3) {
                            slide.toMin();
                            pivotToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 8 && slide.isTimeDone() && timer.seconds() > 0.2) {
                            taskNumber = 0;
                            headingTarget = -180;
                            cycleCount = 3;
                            latchToggle.toTrue();
                            wristToggle = false;
                            intake.setWristUp();
                            camera.openCamera();
                            gvf.setPath(PathList.BasketToSub, 0.5, 5, 0.8, pose);
                            state = ApexStates.CAMERA;
                        }
                    }
                    break;

                case CAMERA:
                    if (taskNumber == 0 && gvf.isDone(4, 7)) {
                        if (pipeline.getSampleThetaValue() != 0) {
                            lastTheta = pipeline.getSampleThetaValue() * (Math.abs(swerve.getHeadingInDegrees()) / 10 + 1);
                        }
                        lastTheta = limiter.rateLimit(lastTheta, 14);
                        if (lastTheta < 7) {
                            taskNumber++;
                        }
                    }
                    else if (taskNumber == 1) {
                        latchToggle.toTrue();
                        intake.setWristOpen();
                        rotatorToggle.toTrue();
                        intake.setSpinIn();
                        taskNumber++;
                    }
                    else if (taskNumber == 2 && timer.seconds() > 1) {
                        rotatorToggle.toFalse();
                        intake.setWristDown();
                        taskNumber++;
                    }
                    else if (taskNumber == 3 && timer.seconds() > 1) {
                        slide.toSetPoint2();
                        if (intake.getColor() == BrushColor.ColorDetection.YELLOW) {
                            latchToggle.toFalse();
                            slide.toMin();
                            intake.setWristUp();
                            taskNumber++;
                        }
                    }
                    else if (taskNumber == 4 && slide.isTimeDone()) {
                        gvf.setPath(PathList.SubToBasket, 0.5, 10, 0.8, pose);
                        intake.setWristDown();
                        pivotToggle.toTrue();
                        headingTarget = -135;
                        lastY = -1;
                        lastTheta = 0;
                        taskNumber++;
                    }
                    else if (taskNumber == 5 && gvf.isDone(4, 7)) {
                        slide.toMax();
                        //intake.setSpin0();
                        taskNumber++;
                    }
                    else if (taskNumber == 6 && slide.isTimeDone()) {
                        rotatorToggle.toTrue();
                        taskNumber++;
                    }
                    else if (taskNumber == 7 && timer.seconds() > 0.5) {
                        latchToggle.toFalse();
                        //intake.setSpinIn();
                        taskNumber++;
                    }
                    else if (taskNumber == 8 && timer.seconds() > 0.4) {
                        pivotClear = true;
                        //intake.setSpin0();
                        rotatorToggle.toFalse();
                        taskNumber++;
                    }
                    else if (taskNumber == 9 && timer.seconds() > 0.3) {
                        slide.toMin();
                        pivotToggle.toFalse();
                        taskNumber++;
                    }
                    else if (taskNumber == 10 && slide.isTimeDone() && timer.seconds() > 0.2) {
                        taskNumber = 0;
                        headingTarget = -90;
                        wristToggle = false;
                        intake.setWristUp();
                        latchToggle.toTrue();
                        gvf.setPath(PathList.BasketToSub, 0.5, 5, 0.6, pose);
                        state = ApexStates.STANDBY;
                    }
                    break;

                case STANDBY:
                    slide.toMin();
                    latchToggle.toFalse();
                    wristToggle = false;
                    intake.setWristUp();
                    intake.setSpin0();
                    pivotToggle.toFalse();
                    break;
            }

            if (latchToggle.isTrue()) {
                intake.setLatchClose();
            }
            else {
                intake.setLatchOpen();
            }

            if (wristToggle) {
                intake.setWristDown();
            }
            else {
                //intake.setWristUp();
            }

            if (rotatorToggle.isTrue()) {
                intake.setRotatorTo180();
            }
            else {
                intake.setRotatorTo0();
            }

            if (pivotToggle.isTrue()) {
                if (pivotClear) {
                    slide.movePivotTo(90);
                }
                else slide.movePivotTo(100);
            }
            else if (slide.getSlidePosition() < 50) {
                slide.movePivotTo(0);
                pivotClear = false;
            }

            //executes at end of every task
            if (lastTaskNumber != taskNumber) {
                lastTaskNumber = taskNumber;
                timer.reset();
            }

            slide.update();

            telemetry.addData("pose",pose.toString());
            telemetry.addData("total arc", gvf.temp3);
            telemetry.addData("poseError",gvf.poseError);
            telemetry.addData("headingError", gvf.headingError);
            telemetry.addData("tasknum",taskNumber);
            telemetry.addData("path minus",PathList.SideToScore.temp);
            telemetry.addData("path arc in gvf",PathList.SideToScore.temp2);
            telemetry.addData("arc",gvf.arcLengthRemaining());
            telemetry.addData("state",state);
            telemetry.addData("wri", wristToggle);
            telemetry.addData("path", Arrays.toString(gvf.getPath().getControlPointList()));
            telemetry.update();
        }
        camera.stopStreaming();
        pipeline.releaseMats();
        camera.setLightPower(0);
    }
}
