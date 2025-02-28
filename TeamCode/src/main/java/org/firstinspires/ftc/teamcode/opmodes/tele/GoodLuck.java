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
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;

import java.util.List;

@Config
@TeleOp(name="GOOD LUCK", group="Linear Opmode")
public class GoodLuck extends LinearOpMode {

    private double headingTarget;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        PID headingPID = new PID(0.2,0.017,0,0.1,5);
        ButtonDetector headingPIDtoggle  = new ButtonDetector();

        PivotingSlide slide = new PivotingSlide(hardwareMap, false);
        ButtonDetector pivotToggle = new ButtonDetector();

        Intake intake = new Intake(hardwareMap);
        ButtonDetector latchToggle = new ButtonDetector();
        ButtonDetector wristToggle = new ButtonDetector();

        ButtonDetector rotatorToggle = new ButtonDetector();

        ElapsedTime hzTimer = new ElapsedTime();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        intake.setRotatorTo0();

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

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation);

            /*
            if (gamepad2.start) {
                wristToggle.toFalse();
                slide.toMin();
            }

            if (gamepad2.back && slide.getPivotAngle() < 50 && slide.getSlidePosition() < 50) {
                pivotToggle.toTrue();
                startAction = true;
            }
            else if (startAction && slide.getPivotAngle() > 72.5) {
                slide.toMax();
            }
            if (startAction && slide.getSlidePosition() > 750) {
                wristToggle.toFalse();
                startAction = false;
            }

             */

            if (latchToggle.toggle(gamepad2.left_bumper)) {
                intake.setLatchOpen();
            }
            else intake.setLatchClose();

            if (gamepad2.dpad_down) {
                intake.setSpinIn();
            }
            else if (gamepad2.dpad_up) {
                intake.setSpinOut();
            }
            else intake.setSpin0();

            if (wristToggle.toggle(gamepad2.left_trigger > 0.2)) {
                intake.setWristDown();
            }
            else {
                intake.setWristUp();
            }

            if (rotatorToggle.toggle(gamepad2.right_trigger > 0.2)) {
                intake.setRotatorTo180();
            }
            else {
                intake.setRotatorTo0();
            }

            if (slide.getSlidePosition() < 50) {
                if (pivotToggle.toggle(gamepad2.right_bumper)) {
                    slide.movePivotTo(100);
                    rotatorToggle.toTrue();
                    wristToggle.toTrue();

                }
                else  {
                    slide.movePivotTo(0);
                    //wristToggle.toFalse();
                    rotatorToggle.toFalse();
                    latchToggle.toFalse();
                }
            }

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

            slide.update();

            telemetry.addData("hz", hzTimer.milliseconds());
            telemetry.addData("peice",Math.toDegrees(Maths.peicewiseAtan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)) - 90);
            telemetry.addData("slied",slide.getSlidePosition());
            telemetry.addData("pivot",slide.getPivotAngle());

            hzTimer.reset();
            telemetry.update();
        }
    }
}

