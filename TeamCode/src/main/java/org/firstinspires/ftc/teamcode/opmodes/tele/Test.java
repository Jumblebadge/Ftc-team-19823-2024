package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.utility.camera.BrushColor;
import org.firstinspires.ftc.teamcode.utility.wrappers.DcMotorExW;

@Config
@TeleOp(name="Test", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static double slideTarget = 0, pivotTarget = 0;

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
        while (opModeIsActive() && !isStopRequested()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            telemetry.addData("millis: ",hztimer.milliseconds());
            telemetry.addData("slide pos", slide.getSlidePosition());
            telemetry.addData("motion", slide.getMotionTarget());
            telemetry.addData("pvit", slide.getPivotAngle());

            slide.moveSlideTo(slideTarget);

            slide.movePivotTo(pivotTarget);

            slide.update();
            hztimer.reset();
            telemetry.update();
        }

    }
}
