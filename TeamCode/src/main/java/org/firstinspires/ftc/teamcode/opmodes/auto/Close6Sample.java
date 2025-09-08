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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.DashOperations;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;
import org.firstinspires.ftc.teamcode.utility.PathList;
import org.firstinspires.ftc.teamcode.utility.camera.BrushColor;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name="Close 6 Sample", group="Linear Opmode")
public class Close6Sample extends LinearOpMode {

    private double headingTarget = 0;
    private int cycleCount = 0;
    private int taskNumber = 0, lastTaskNumber = 0;
    boolean pivotClear = false;
    boolean wristToggle = true;
    boolean wristUp = false;
    ElapsedTimeW timer = new ElapsedTimeW();
    ElapsedTimeW color = new ElapsedTimeW();

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

            double rotation = gvf.headingOut(headingTarget,swerve.getHeadingInDegrees());
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
                        rotatorToggle.toTrue();
                        taskNumber++;
                    }
                    else if (taskNumber == 2 && slide.isSlideTimeDone()) {
                        wristToggle = false;
                        //intake.setSpin0();
                        taskNumber++;
                    }
                    else if (taskNumber == 3 && timer.seconds() > 0.075) {
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
                    else if (taskNumber == 6 && slide.isSlideTimeDone(-0.3) && timer.seconds() > 0.15) {
                        taskNumber = 0;
                        headingTarget = -103;
                        state = ApexStates.CYCLE;
                        wristToggle = true;
                        latchToggle.toTrue();
                    }
                    break;
                case CYCLE:
                    if (cycleCount == 0) {
                        if (taskNumber == 0 && slide.isSlideTimeDone() && gvf.isDone(4, 7) && slide.isPivotPositionDone()) {
                            slide.toSetPoint3();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && slide.isSlideTimeDone()) {
                            slide.toMin();
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && slide.isSlideTimeDone()) {
                            pivotToggle.toTrue();
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && gvf.isDone(4, 7) && slide.isPivotPositionDone()) {
                            slide.toMax();
                            rotatorToggle.toTrue();
                            //intake.setSpin0();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && slide.isSlideTimeDone()) {
                            wristToggle = false;
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.075) {
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
                        else if (taskNumber == 8 && slide.isSlideTimeDone(-0.3) && timer.seconds() > 0.15) {
                            taskNumber = 0;
                            headingTarget = -85;
                            cycleCount = 1;
                            wristToggle = true;
                            latchToggle.toTrue();
                        }
                    }
                    else if (cycleCount == 1) {
                        if (taskNumber == 0 && slide.isSlideTimeDone() && gvf.isDone(4, 7) && slide.isPivotPositionDone()) {
                            slide.toSetPoint3();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && slide.isSlideTimeDone()) {
                            slide.toMin();
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && slide.isSlideTimeDone()) {
                            pivotToggle.toTrue();
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && gvf.isDone(4, 7) && slide.isPivotPositionDone()) {
                            slide.toMax();
                            rotatorToggle.toTrue();
                            //intake.setSpin0();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && slide.isSlideTimeDone()) {
                            wristToggle = false;
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.075) {
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
                        else if (taskNumber == 8 && slide.isSlideTimeDone(-0.3) && timer.seconds() > 0.15) {
                            taskNumber = 0;
                            headingTarget = -69;
                            cycleCount = 2;
                            wristToggle = true;
                            latchToggle.toTrue();
                        }
                    }
                    else if (cycleCount == 2) {
                        if (taskNumber == 0 && slide.isSlideTimeDone() && gvf.isDone(4, 7) && slide.isPivotPositionDone()) {
                            slide.toSetPoint3();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && slide.isSlideTimeDone()) {
                            slide.toMin();
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && slide.isSlideTimeDone()) {
                            pivotToggle.toTrue();
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && gvf.isDone(4, 7) && slide.isPivotPositionDone()) {
                            slide.toMax();
                            rotatorToggle.toTrue();
                            //intake.setSpin0();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && slide.isSlideTimeDone()) {
                            wristToggle = false;
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.075) {
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
                        else if (taskNumber == 8 && slide.isSlideTimeDone(-0.3)) {
                            taskNumber = 0;
                            headingTarget = -180;
                            cycleCount = 0;
                            latchToggle.toTrue();
                            wristToggle = false;
                            wristUp = true;
                            intake.setWristUp();
                            gvf.setPath(PathList.BasketToSub, 0.5, 5, 0.8, pose);
                            state = ApexStates.CAMERA;
                        }
                    }
                    break;

                case CAMERA:
                    if (taskNumber == 0 && gvf.isDone(4, 7)) {
                        if (cycleCount < 1) {
                            rotatorToggle.toTrue();
                            taskNumber++;
                        }
                        else {
                            taskNumber = 3;
                            latchToggle.toTrue();
                            rotatorToggle.toFalse();
                            intake.setSpinIn();
                            wristToggle = true;
                        }
                        wristUp = false;
                    }
                    else if (taskNumber == 1) {
                        headingTarget = -165;
                        latchToggle.toTrue();
                        intake.setSpinIn();
                        taskNumber++;
                    }
                    else if (taskNumber == 2 && timer.seconds() > 0.5) {
                        headingTarget = -180;
                        rotatorToggle.toFalse();
                        wristToggle = true;
                        color.reset();
                        taskNumber++;
                    }
                    else if (taskNumber == 3 && timer.seconds() > 0.5) {
                        headingTarget = -180;
                        if (color.seconds() > 0.6) {
                            color.reset();
                            slide.toSetPoint1();
                        }
                        if (slide.isSlideTimeDone() && intake.getColor() == BrushColor.ColorDetection.NONE) {
                            if (timer.seconds() > 2) {
                                slide.moveSlideTo(420);
                            }
                            else slide.toSetPoint2();
                        }
                        if (intake.getColor() == BrushColor.ColorDetection.RED) {
                            latchToggle.toFalse();
                        }
                        else latchToggle.toTrue();
                        if (!(intake.getColor() == BrushColor.ColorDetection.YELLOW || intake.getColor() == BrushColor.ColorDetection.BLUE)) {
                            color.reset();
                        }
                        if (color.seconds() > 0.15) {
                            latchToggle.toTrue();
                            slide.toMin();
                            wristToggle = false;
                            taskNumber++;
                        }
                    }
                    else if (taskNumber == 4 && slide.isSlideTimeDone()) {
                        gvf.setPath(PathList.SubToBasket, 0.5, 10, 0.8, pose);
                        wristUp = true;
                        pivotToggle.toTrue();
                        headingTarget = -135;
                        taskNumber++;
                    }
                    else if (taskNumber == 5 && timer.seconds() > 1) {
                        wristToggle = true;
                        wristUp = false;
                        if (gvf.isDone(4, 7) && slide.isPivotPositionDone()) {
                            slide.toMax();
                            //intake.setSpin0();
                            taskNumber++;
                        }
                    }
                    else if (taskNumber == 6 && slide.isSlideTimeDone()) {
                        rotatorToggle.toTrue();
                        taskNumber++;
                    }
                    else if (taskNumber == 7 && timer.seconds() > 0.15) {
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
                    else if (taskNumber == 10 && slide.isSlideTimeDone(-0.3) && timer.seconds() > 0.15) {
                        taskNumber = 0;
                        cycleCount++;
                        //headingTarget = -180;
                        latchToggle.toTrue();
                        wristToggle = false;
                        wristUp = true;
                        intake.setWristUp();
                        //gvf.setPath(PathList.BasketToSub, 0.5, 5, 0.8, pose);
                        state = ApexStates.STANDBY;
                    }
                    break;

                case STANDBY:
                    slide.toMin();
                    headingTarget = -90;
                    latchToggle.toFalse();
                    wristToggle = false;
                    wristUp = true;
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
                if (wristUp) intake.setWristUp();
                else intake.setWristMiddle();
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
            telemetry.addData("color", intake.getColor());
            telemetry.addData("path minus",PathList.SideToScore.temp);
            telemetry.addData("path arc in gvf",PathList.SideToScore.temp2);
            telemetry.addData("arc",gvf.arcLengthRemaining());
            telemetry.addData("state",state);
            telemetry.addData("wri", wristToggle);
            telemetry.addData("path", Arrays.toString(gvf.getPath().getControlPointList()));
            telemetry.update();
        }
    }
}
