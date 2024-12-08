package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.ThreeAxisClaw;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

import java.util.List;

@Config
@TeleOp(name="GOOD LUCK", group="Linear Opmode")
public class GoodLuck extends LinearOpMode {

    private double headingTarget;
    private double nanoTime;

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

        double slideTarget = 0;
        DcMotorExW liftLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Llift"));
        DcMotorExW liftRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Rlift"));

        MotorGroup slideMotors = new MotorGroup(liftLeft, liftRight);

        RunMotionProfile profile = new RunMotionProfile(37500,42500,42500,new ConstantsForPID(0.2,0,0.2,0.2,2,0));

        slideMotors.resetEncoders();

        ThreeAxisClaw claw = new ThreeAxisClaw(hardwareMap);
        ButtonDetector clawToggle = new ButtonDetector();
        ButtonDetector wristToggle = new ButtonDetector();

        ElapsedTime hzTimer = new ElapsedTime();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        waitForStart();
        while (opModeIsActive()) {
            previous1.copy(current1);
            current1.copy(gamepad1);
            previous2.copy(current2);
            current2.copy(gamepad2);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double rotation;
            if (headingPIDtoggle.toggle(gamepad1.right_bumper)) {
                rotation = headingPID.pidAngleOut(headingTarget, swerve.getHeadingInDegrees());
                gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            }
            else {
                rotation = gamepad1.right_stick_x;
                gamepad1.setLedColor(0,1,0,Gamepad.LED_DURATION_CONTINUOUS);
            }

            if (current1.left_bumper && !previous1.left_bumper) {
                headingTarget = swerve.getHeadingInDegrees();
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
                headingTarget = Maths.peicewiseAtan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            }

            swerve.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, rotation);

            if (clawToggle.toggle(gamepad2.left_bumper)) {
                claw.setClawClose();
            }
            else claw.setClawOpen();

            if (wristToggle.toggle(gamepad2.left_trigger > 0.2)) {
                claw.setWristDown();
            }
            else claw.setWristUp();

            if (pivotToggle.toggle(gamepad2.right_bumper)) {
                slide.movePivotTo(85);
            }
            else if (slideMotors.getPosition(0) < 50) {
                slide.movePivotTo(0);
            }

            slideMotors.setPowers(profile.profiledMovement(slideTarget, slideMotors.getPosition(0)));

            if (gamepad2.triangle && slide.getPivotAngle() > 20) {
                slideTarget = 900;
            }
            if (gamepad2.circle) {
                slideTarget = 600;
            }
            if (gamepad2.square) {
                slideTarget = 300;
            }
            if (gamepad2.cross) {
                slideTarget = 0;
                pivotToggle.toFalse();
            }

            slide.update();

            telemetry.addData("hz", 1000000000 / (System.nanoTime() - nanoTime));
            telemetry.addData("motion",profile.getMotionTarget());
            telemetry.addData("current", slideMotors.getPosition(0));
            nanoTime = System.nanoTime();

            telemetry.update();
        }
    }
}

