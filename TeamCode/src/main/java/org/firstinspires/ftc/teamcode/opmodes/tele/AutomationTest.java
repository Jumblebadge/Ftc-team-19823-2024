package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;
import org.firstinspires.ftc.teamcode.utility.wrappers.ServoImplExW;

import java.util.List;

@Config
@TeleOp(name="autlomation test", group="Linear Opmode")
public class AutomationTest extends LinearOpMode {

    private double headingTarget;

    boolean wristMiddle = false;

    private enum States {
        MANUAL,
        HORIZONTAL_EXTENSION,
        HORIZONTAL,
        INTAKE,
        HORIZONTAL_RETRACTION,
        PIVOT_STANDBY,
        SCORE,
        LATCH,
        VERTICAL_RETRACTION,
        STANDBY
    }

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        PID headingPID = new PID(0.09,0.00188,0,0.025,1);
        ButtonDetector headingPIDtoggle  = new ButtonDetector();

        PivotingSlide slide = new PivotingSlide(hardwareMap);
        ButtonDetector pivotToggle = new ButtonDetector();
        ButtonDetector hangToggle = new ButtonDetector();

        Intake intake = new Intake(hardwareMap);
        ButtonDetector latchToggle = new ButtonDetector();
        ButtonDetector wristToggle = new ButtonDetector();
        ButtonDetector spinToggle  = new ButtonDetector(true);
        ButtonDetector melonReverse = new ButtonDetector();
        ButtonDetector rotatorToggle = new ButtonDetector();

        ElapsedTimeW timer = new ElapsedTimeW();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        States state = States.STANDBY;
        States lastState = state;

        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            previous1.copy(current1);
            current1.copy(gamepad1);
            previous2.copy(current2);
            current2.copy(gamepad2);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double heading = swerve.getJustHeadingInDegrees();

            double multiplier = 1 - gamepad1.right_trigger * 0.4;

            double rotation;
            if (headingPIDtoggle.toggle(gamepad1.right_bumper)) {
                rotation = headingPID.pidAngleOut(headingTarget, heading);
                gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            }
            else {
                rotation = -gamepad1.right_stick_x * multiplier / 2;
                gamepad1.setLedColor(0,1,0,Gamepad.LED_DURATION_CONTINUOUS);
            }

            if (current1.left_bumper && !previous1.left_bumper) {
                headingTarget = heading;
                headingPIDtoggle.toTrue();
            }

            if (gamepad1.options) {
                swerve.resetIMU();
            }

            if (gamepad1.share) {
                swerve.resetPoseAndHeading();
            }

            if (current1.dpad_up && !previous1.dpad_up) {
                headingTarget = 0;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_left && !previous1.dpad_left) {
                headingTarget = 90;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_down && !previous1.dpad_down) {
                headingTarget = 180;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_right && !previous1.dpad_right) {
                headingTarget = -90;
                headingPIDtoggle.toTrue();
            }
            if (Math.sqrt(Math.pow(gamepad1.right_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2)) > 0.25) {
                headingTarget = Math.toDegrees(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)) - 90;
            }

            swerve.drive(-gamepad1.left_stick_x * multiplier, -gamepad1.left_stick_y * multiplier, rotation);

            if (current1.ps || current2.y && !previous2.y) state = States.STANDBY;

            if (gamepad2.a) state = States.SCORE;

            melonReverse.toggleVoid(gamepad2.b);

            switch (state) {
                case MANUAL:

                    latchToggle.toggleVoid(gamepad2.left_trigger > 0.2);

                    wristToggle.toggleVoid(gamepad2.right_trigger > 0.2);

                    pivotToggle.toggleVoid(gamepad2.right_bumper);

                    if (gamepad2.triangle && slide.getPivotAngle() > 30) {
                        slide.toMax();
                    }
                    if (gamepad2.circle) {
                        slide.toSetPoint2();
                    }
                    if (gamepad2.square) {
                        slide.toSetPoint1();
                    }
                    if (gamepad2.cross) {
                        slide.toMin();
                        pivotToggle.toFalse();
                    }

                    break;
                case STANDBY:

                    slide.toMin();
                    latchToggle.toFalse();
                    wristToggle.toFalse();
                    pivotToggle.toFalse();
                    spinToggle.toFalse();
                    wristMiddle = false;

                    if (current2.right_bumper && !previous2.right_bumper) state = States.HORIZONTAL_EXTENSION;
                    if (current2.x && !previous2.x) state = States.HORIZONTAL;

                    break;

                case HORIZONTAL:

                    rotatorToggle.toFalse();
                    wristToggle.toTrue();
                    spinToggle.toTrue();

                    latchToggle.toggleVoid(gamepad2.right_trigger > 0.2);

                    if (current2.right_bumper && !previous2.right_bumper) state = States.HORIZONTAL_RETRACTION;

                    break;
                case HORIZONTAL_EXTENSION:

                    wristMiddle = true;
                    slide.toSetPoint2();
                    rotatorToggle.toFalse();

                    if (current2.right_bumper && !previous2.right_bumper) state = States.INTAKE;

                    break;
                case INTAKE:

                    wristMiddle = false;
                    wristToggle.toTrue();
                    spinToggle.toTrue();
                    rotatorToggle.toFalse();

                    latchToggle.toggleVoid(gamepad2.right_trigger > 0.2);

                    if (current2.right_bumper && !previous2.right_bumper) state = States.HORIZONTAL_RETRACTION;

                    break;
                case HORIZONTAL_RETRACTION:

                    slide.toMin();
                    //spinToggle.toFalse();
                    latchToggle.toFalse();
                    rotatorToggle.toTrue();
                    wristToggle.toFalse();

                    if (current2.right_bumper && !previous2.right_bumper) state = States.PIVOT_STANDBY;

                    break;
                case PIVOT_STANDBY:

                    pivotToggle.toTrue();
                    if (slide.getPivotAngle() > 10) {
                        wristToggle.toTrue();
                    }
                    rotatorToggle.toTrue();

                    if (current2.right_bumper && !previous2.right_bumper) state = States.SCORE;

                    break;

                case SCORE:

                    slide.toMax();
                    pivotToggle.toTrue();

                    if (slide.isPositionDone()) {
                        wristToggle.toFalse();
                        wristMiddle = true;
                    }

                    if (current2.right_bumper && !previous2.right_bumper) state = States.LATCH;

                    break;

                case LATCH:

                    latchToggle.toTrue();
                    spinToggle.toTrue();

                    if (current2.right_bumper && !previous2.right_bumper) state = States.VERTICAL_RETRACTION;

                    break;

                case VERTICAL_RETRACTION:

                    latchToggle.toFalse();
                    spinToggle.toFalse();

                    if (timer.seconds() < 0.25) {
                        wristToggle.toTrue();
                        wristMiddle = false;
                    }
                    else {
                        slide.toMin();
                        pivotToggle.toFalse();
                    }

                    if (slide.getPivotAngle() < 20) {
                        wristToggle.toFalse();
                        state = States.STANDBY;
                    }
                    if (current2.start && !previous2.start) state = States.SCORE;

                    break;
            }


            if (latchToggle.isTrue()) {
                intake.setLatchOpen();
            }
            else intake.setLatchClose();

            if (spinToggle.isTrue()) {
                if (melonReverse.isTrue()) intake.setSpinOut();
                else intake.setSpinIn();
            }
            else intake.setSpin0();

            if (wristToggle.isTrue()) {
                intake.setWristDown();
            }
            else if (wristMiddle) {
                intake.setWristMiddle();
            }
            else {
                intake.setWristUp();
            }

            if (rotatorToggle.isTrue()) {
                intake.setRotatorTo180();
            }
            else {
                intake.setRotatorTo0();
            }

            if (hangToggle.toggle(gamepad2.right_stick_button && gamepad2.left_stick_button)) {
                state = States.MANUAL;
                slide.toMin();
                slide.movePivotTo(3);
            }
            else {
                if (slide.getSlidePosition() < 50) {
                    if (pivotToggle.isTrue()) {
                        slide.movePivotTo(100);
                    }
                    else  {
                        slide.movePivotTo(0);
                    }
                }
            }

            if (lastState != state) {
                lastState = state;
                timer.reset();
                melonReverse.toFalse();
            }

            slide.update();

            telemetry.addData("state", state);
            telemetry.addData("heading", heading);
            telemetry.addData("headingT", headingTarget);
            telemetry.update();
        }
    }
}