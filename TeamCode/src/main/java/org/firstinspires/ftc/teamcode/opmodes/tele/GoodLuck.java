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
import org.firstinspires.ftc.teamcode.subsystems.ThreeAxisClaw;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;

import java.util.List;

@Config
@TeleOp(name="GOOD LUCK", group="Linear Opmode")
public class GoodLuck extends LinearOpMode {

    private double headingTarget;
    private double slideTarget;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        PID headingPID = new PID(0.09,0.00188,0,0.025,1);
        ButtonDetector headingPIDtoggle  = new ButtonDetector();

        ButtonDetector startToggle = new ButtonDetector();

        PivotingSlide slide = new PivotingSlide(hardwareMap);
        ButtonDetector pivotToggle = new ButtonDetector();

        ThreeAxisClaw claw = new ThreeAxisClaw(hardwareMap);
        ButtonDetector clawToggle = new ButtonDetector();
        ButtonDetector wristToggle = new ButtonDetector();

        ButtonDetector rotatorToggle = new ButtonDetector();
        boolean lastRotatorToggle = false;

        ElapsedTime hzTimer = new ElapsedTime();
        ElapsedTime rotatorTimer = new ElapsedTime();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        claw.setRotatorTo0();

        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        waitForStart();
        while (opModeIsActive()) {
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

            /**
            if (startToggle.toggle(gamepad2.start)) {
                if (slide.getSlidePosition() < 50 && slide.getPivotAngle() < 20) ;
            }
            if (slide.getSlidePosition() < 50 && slide.getPivotAngle() < 20 && startToggle.toggle(gamepad2.start)) {
                if (slide.getPivotAngle() > 75) {
                    slide.toMax();
                }
                wristToggle.toFalse();
                pivotToggle.toTrue();
            }
             */

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation);

            if (clawToggle.toggle(gamepad2.left_bumper)) {
                claw.setClawClose();
            }
            else claw.setClawOpen();

            if (wristToggle.toggle(gamepad2.left_trigger > 0.2)) {
                if (rotatorToggle.toggle(gamepad2.right_trigger > 0.2)) {
                    claw.setRotatorTo90();
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
                if (pivotToggle.toggle(gamepad2.right_bumper)) {
                    slide.movePivotTo(85);
                }
                else  {
                    slide.movePivotTo(0);
                }
            }

            if (gamepad2.triangle && slide.getPivotAngle() > 20) {
                slideTarget = 830;
            }
            if (gamepad2.circle) {
                slideTarget = 550;
            }
            if (gamepad2.square) {
                slideTarget = 300;
            }
            if (gamepad2.cross) {
                slideTarget = 0;
                pivotToggle.toFalse();
            }

            slide.moveSlideTo(slideTarget);
            slide.update();

            telemetry.addData("hz", hzTimer.milliseconds());
            telemetry.addData("peice",Math.toDegrees(Maths.peicewiseAtan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)) - 90);
            telemetry.addData("slidetarget",slideTarget);
            telemetry.addData("slied",slide.getSlidePosition());

            hzTimer.reset();
            telemetry.update();
        }
    }
}

