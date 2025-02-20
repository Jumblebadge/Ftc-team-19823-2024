package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.ServoImplExW;

@Config
@TeleOp(name="test", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static double target = 0, Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 0, maxVel = 1, maxAccel = 1, maxJerk = 1;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        PivotingSlide slide = new PivotingSlide(hardwareMap, false);

        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {

            slide.setMotionConstraints(maxVel, maxAccel, maxJerk);
            slide.setPidConstants(new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, 0));

            slide.moveSlideTo(target);
            slide.update();
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            telemetry.addData("millis",hztimer.milliseconds());
            telemetry.addData("slidepos", slide.getSlidePosition());
            telemetry.addData("motiontarget", slide.getMotionTarget());
            hztimer.reset();
            telemetry.update();
        }
    }
}
