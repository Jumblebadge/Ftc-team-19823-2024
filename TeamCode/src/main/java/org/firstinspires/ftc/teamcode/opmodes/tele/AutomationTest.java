package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;

import java.util.List;

@Config
@TeleOp(name="autlomation test", group="Linear Opmode")
public class AutomationTest extends LinearOpMode {

    private double headingTarget;

    private enum States {
        MANUAL,
        INTAKE,
        HORIZONTAL_RETRACTION,
        PIVOT_STANDBY,
        SCORE,
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

        PivotingSlide slide = new PivotingSlide(hardwareMap, false);
        ButtonDetector pivotToggle = new ButtonDetector();

        Intake claw = new Intake(hardwareMap);
        ButtonDetector clawToggle = new ButtonDetector();
        ButtonDetector wristToggle = new ButtonDetector();
        ButtonDetector rotatorToggle = new ButtonDetector();
        boolean lastRotatorToggle = false;
        ElapsedTimeW rotatorTimer = new ElapsedTimeW();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        States state = States.STANDBY;

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

            double rotation;
            if (headingPIDtoggle.toggle(gamepad1.right_bumper)) {
                rotation = headingPID.pidAngleOut(headingTarget, heading);
                gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            }
            else {
                rotation = -gamepad1.right_stick_x;
                gamepad1.setLedColor(0,1,0,Gamepad.LED_DURATION_CONTINUOUS);
            }

            if (current1.left_bumper && !previous1.left_bumper) {
                headingTarget = heading;
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

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation);

            if (current1.share) state = States.MANUAL;

            switch (state) {
                case MANUAL:

                    clawToggle.toggleVoid(gamepad2.left_trigger > 0.2);

                    wristToggle.toggleVoid(gamepad2.right_trigger > 0.2);

                    pivotToggle.toggleVoid(gamepad2.right_bumper);

                    if (gamepad2.triangle && slide.getPivotAngle() > 20) {
                        slide.toMax();
                    }
                    if (gamepad2.circle) {
                        slide.toSetPoint3();
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
                    clawToggle.toFalse();
                    wristToggle.toFalse();
                    pivotToggle.toFalse();

                    if (gamepad1.right_bumper) state = States.INTAKE;

                    break;
                case INTAKE:

                    if (gamepad2.circle) {
                        slide.toSetPoint3();
                    }
                    if (gamepad2.square) {
                        slide.toSetPoint1();
                    }
                    if (gamepad2.cross) {
                        slide.toMin();
                    }

                    clawToggle.toggleVoid(gamepad1.left_trigger > 0.2);

                    wristToggle.toggleVoid(gamepad1.right_trigger > 0.2);

                    if (gamepad1.right_bumper) state = States.HORIZONTAL_RETRACTION;
                    if (gamepad1.left_bumper) state = States.STANDBY;

                    break;
                case HORIZONTAL_RETRACTION:

                    wristToggle.toFalse();
                    slide.toMin();

                    if (gamepad1.right_bumper) state = States.PIVOT_STANDBY;
                    if (gamepad1.left_bumper) state = States.INTAKE;

                    break;
                case PIVOT_STANDBY:

                    pivotToggle.toTrue();

                    if (gamepad1.right_bumper) state = States.SCORE;
                    if (gamepad1.left_bumper) state = States.HORIZONTAL_RETRACTION;

                    break;

                case SCORE:

                    slide.toMax();

                    clawToggle.toggleVoid(gamepad1.left_trigger > 0.2);

                    wristToggle.toggleVoid(gamepad1.right_trigger > 0.2);

                    if (gamepad1.right_bumper) state = States.VERTICAL_RETRACTION;
                    if (gamepad1.left_bumper) state = States.PIVOT_STANDBY;

                    break;

                case VERTICAL_RETRACTION:

                    clawToggle.toFalse();
                    wristToggle.toTrue();

                    slide.toMin();
                    pivotToggle.toFalse();

                    if (slide.getPivotAngle() < 20) state = States.STANDBY;
                    if (gamepad1.left_bumper) state = States.SCORE;

                    break;
            }

            if (state == States.MANUAL) {
                gamepad2.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                gamepad1.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            }
            else {
                gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            }

            if (clawToggle.isTrue()) {
                //claw.setClawClose();
            }
            else //claw.setClawOpen();

            if (wristToggle.toggle(gamepad2.left_trigger > 0.2)) {
                if (rotatorToggle.toggle(gamepad2.right_trigger > 0.2)) {
                    claw.setRotatorTo180();
                }
                else {
                    claw.setRotatorTo0();
                }
                if (rotatorTimer.seconds() < 0.2 && slide.getSlidePosition() < 50) {
                    claw.setWristClear();
                }
                else {
                    claw.setWristDown();
                }
            }
            else {
                claw.setWristUp();
                claw.setRotatorTo0();
                rotatorToggle.toFalse();
            }

            if (rotatorToggle.isTrue() != lastRotatorToggle) {
                lastRotatorToggle = rotatorToggle.isTrue();
                rotatorTimer.reset();
            }

            if (slide.getSlidePosition() < 50) {
                if (pivotToggle.isTrue()) {
                    slide.movePivotTo(85);
                }
                else  {
                    slide.movePivotTo(0);
                }
            }

            slide.update();

            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}

