package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.camera.CameraShenanigans;
import org.firstinspires.ftc.teamcode.utility.camera.TestPipeline;
import org.firstinspires.ftc.teamcode.utility.camera.YellowECircle;

import java.util.List;

@Config
@TeleOp(name="test ", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 1, maxVel = 1, maxAccel = 1, maxJerk = 1, target = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        PivotingSlide slide = new PivotingSlide(hardwareMap, false);

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

            slide.moveSlideTo(target);

            slide.update();

            telemetry.addData("hstimes", hzTimer.milliseconds());
            telemetry.addData("motionstate", slide.getMotionTarget());
            telemetry.addData("tar", target);
            telemetry.addData("pivot", slide.getSlidePosition());
            hzTimer.reset();
            telemetry.update();

        }
    }
}

