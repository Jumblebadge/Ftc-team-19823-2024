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
@Autonomous(name="Blue Left Yellow", group="Linear Opmode")
public class BlueLeftYellow extends LinearOpMode {

    private double headingTarget = 0;
    private int cycleCount = 0;
    private int taskNumber = 0, lastTaskNumber = 0;
    boolean pivotClear = false;
    ElapsedTimeW timer = new ElapsedTimeW();

    private enum ApexStates {
        PRELOAD,
        CYCLE,
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

        GVF gvf = new GVF(PathList.BlueSideToScore, 0.5, 5, 0.7, telemetry);

        PivotingSlide slide = new PivotingSlide(hardwareMap, true);
        ButtonDetector pivotToggle = new ButtonDetector();

        ThreeAxisClaw claw = new ThreeAxisClaw(hardwareMap);
        ButtonDetector clawToggle = new ButtonDetector(true);
        claw.setClawClose();
        claw.setRotatorTo0();
        ButtonDetector wristToggle = new ButtonDetector(true);

        ApexStates state = ApexStates.PRELOAD;

        //Bulk sensor reads
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        waitForStart();

        swerve.setPosition(new Pose2d(31.5, 65));

        while (opModeIsActive()) {

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
                        headingTarget = -135;
                        taskNumber++;
                    }
                    else if (taskNumber == 1 && gvf.isDone(4, 7)) {
                        slide.toMax();
                        taskNumber++;
                    }
                    else if (taskNumber == 2 && slide.isTimeDone()) {
                        wristToggle.toFalse();
                        taskNumber++;
                    }
                    else if (taskNumber == 3 && timer.seconds() > 0.3) {
                        clawToggle.toFalse();
                        taskNumber++;
                    }
                    else if (taskNumber == 4 && timer.seconds() > 0.2) {
                        pivotClear = true;
                        taskNumber++;
                    }
                    else if (taskNumber == 5 && timer.seconds() > 0.3) {
                        slide.toMin();
                        pivotToggle.toFalse();
                        taskNumber++;
                    }
                    else if (taskNumber == 6 && slide.isTimeDone()) {
                        taskNumber = 0;
                        headingTarget = -90;
                        state = ApexStates.CYCLE;
                        wristToggle.toTrue();
                        gvf.setPath(PathList.BlueBasketToRightYellow, 0.7, 15, 0.7, pose);
                    }
                    break;
                case CYCLE:
                    if (cycleCount == 0) {
                        if (taskNumber == 0 && gvf.isDone(3.75, 7)) {
                            clawToggle.toTrue();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && timer.seconds() > 0.3) {
                            pivotToggle.toTrue();
                            gvf.setPath(PathList.RightYellowToBlueBasket, 0.7, 5, 0.7, pose);
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && gvf.isDone(4, 7)) {
                            slide.toMax();
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && slide.isTimeDone()) {
                            wristToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && timer.seconds() > 0.3) {
                            clawToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.2) {
                            pivotClear = true;
                            taskNumber++;
                        }
                        else if (taskNumber == 6 && timer.seconds() > 0.3) {
                            slide.toMin();
                            pivotToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 7 && slide.isTimeDone()) {
                            taskNumber = 0;
                            headingTarget = -90;
                            cycleCount = 1;
                            wristToggle.toTrue();
                            gvf.setPath(PathList.BlueBasketToMidYellow, 0.7, 15, 0.7, pose);
                        }
                    }
                    else if (cycleCount == 1) {
                        if (taskNumber == 0 && gvf.isDone(3.75, 7)) {
                            clawToggle.toTrue();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && timer.seconds() > 0.3) {
                            pivotToggle.toTrue();
                            gvf.setPath(PathList.MidYellowToBlueBasket, 0.7, 15, 0.7, pose);
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && gvf.isDone(4, 7) && timer.seconds() > 1) {
                            slide.toMax();
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && slide.isTimeDone()) {
                            wristToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && timer.seconds() > 0.3) {
                            clawToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.2) {
                            pivotClear = true;
                            taskNumber++;
                        }
                        else if (taskNumber == 6 && timer.seconds() > 0.3) {
                            slide.toMin();
                            pivotToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 7 && slide.isTimeDone()) {
                            taskNumber = 0;
                            headingTarget = 0;
                            cycleCount = 2;
                            wristToggle.toTrue();
                            gvf.setPath(PathList.BlueBasketToLeftYellow, 0.7, 15, 0.7, pose);
                        }
                    }
                    else if (cycleCount == 2) {
                        if (taskNumber == 0 && timer.seconds() < 0.5) {
                            claw.setRotatorTo90();
                            taskNumber++;
                        }
                        else if (taskNumber == 1 && gvf.isDone(3.75, 7)) {
                            clawToggle.toTrue();
                            taskNumber++;
                        }
                        else if (taskNumber == 2 && timer.seconds() > 0.3) {
                            claw.setRotatorTo0();
                            pivotToggle.toTrue();
                            gvf.setPath(PathList.MidYellowToBlueBasket, 0.7, 15, 0.7, pose);
                            headingTarget = -135;
                            taskNumber++;
                        }
                        else if (taskNumber == 3 && gvf.isDone(4, 7) && timer.seconds() > 1) {
                            slide.toMax();
                            taskNumber++;
                        }
                        else if (taskNumber == 4 && slide.isTimeDone()) {
                            wristToggle.toFalse();
                            claw.setWristUp();
                            taskNumber++;
                        }
                        else if (taskNumber == 5 && timer.seconds() > 0.3) {
                            clawToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 6 && timer.seconds() > 0.2) {
                            pivotClear = true;
                            taskNumber++;
                        }
                        else if (taskNumber == 7 && timer.seconds() > 0.3) {
                            slide.toMin();
                            pivotToggle.toFalse();
                            taskNumber++;
                        }
                        else if (taskNumber == 8 && slide.isTimeDone()) {
                            taskNumber = 0;
                            headingTarget = -90;
                            cycleCount = 3;
                            wristToggle.toTrue();
                            state = ApexStates.STANDBY;
                        }
                    }
                    break;
                case STANDBY:
                    slide.toMin();
                    clawToggle.toFalse();
                    wristToggle.toFalse();
                    pivotToggle.toFalse();
                    break;
            }


            if (clawToggle.isTrue()) {
                claw.setClawClose();
            }
            else claw.setClawOpen();

            if (wristToggle.isTrue()) {
                claw.setWristDown();
            }
            else claw.setWristUp();

            if (pivotToggle.isTrue()) {
                if (pivotClear) {
                    slide.movePivotTo(80);
                }
                else slide.movePivotTo(85);
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
            telemetry.addData("path minus",PathList.BlueSideToScore.temp);
            telemetry.addData("path arc in gvf",PathList.BlueSideToScore.temp2);
            telemetry.addData("arc",gvf.arcLengthRemaining());
            telemetry.addData("state",state);
            telemetry.addData("path", Arrays.toString(gvf.getPath().getControlPointList()));
            telemetry.update();
        }
    }
}
