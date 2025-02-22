package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.camera.CameraShenanigans;
import org.firstinspires.ftc.teamcode.utility.camera.TestPipeline;

@Config
@TeleOp(name="test", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static boolean on = true;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        TestPipeline pipe = new TestPipeline();

        CameraShenanigans camera = new CameraShenanigans(hardwareMap, dashboard, pipe);

        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            camera.enableProcessor(on);

            telemetry.addData("millis",hztimer.milliseconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
