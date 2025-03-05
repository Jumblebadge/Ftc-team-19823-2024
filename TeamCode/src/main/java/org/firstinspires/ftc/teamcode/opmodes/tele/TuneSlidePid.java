package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.utility.wrappers.ServoImplExW;

import java.util.List;

@Config
@TeleOp(name="tune slide ", group="Linear Opmode")
public class TuneSlidePid extends LinearOpMode {

    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 1, maxVel = 1, maxAccel = 1, maxJerk = 1, slideTarget = 0, pivotTarget = 0, wristTarget = 0.5;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        PivotingSlide slide = new PivotingSlide(hardwareMap, false);

        ServoImplExW servo = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "wrist"));

        ElapsedTime hzTimer = new ElapsedTime();
        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            previous1.copy(current1);
            current1.copy(gamepad1);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            slide.setPidConstants(new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, 0));
            slide.setMotionConstraints(maxVel, maxAccel, maxJerk);

            slide.moveSlideTo(slideTarget);
            slide.movePivotTo(pivotTarget);

            slide.update();

            servo.setPosition(wristTarget);

            telemetry.addData("hstimes", hzTimer.milliseconds());
            telemetry.addData("motionstate", slide.getMotionTarget());
            telemetry.addData("tar", pivotTarget);
            telemetry.addData("pivot", slide.getPivotAngle());
            hzTimer.reset();
            telemetry.update();

        }
    }
}

