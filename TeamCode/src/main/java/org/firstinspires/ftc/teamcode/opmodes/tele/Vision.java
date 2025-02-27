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
import org.firstinspires.ftc.teamcode.utility.camera.CameraShenanigans;
import org.firstinspires.ftc.teamcode.utility.camera.TestPipeline;
import org.firstinspires.ftc.teamcode.utility.camera.YellowECircle;

import java.util.List;

@Config
@TeleOp(name="vision ", group="Linear Opmode")
public class Vision extends LinearOpMode {

    private double headingTarget = 0, lastY = -1;
    public static boolean test = false;
    public static double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        PID headingPID = new PID(0.09,0.00188,0,0.025,1);
        ButtonDetector adjustToggle  = new ButtonDetector();

        YellowECircle pipe = new YellowECircle(telemetry);
        TestPipeline pp = new TestPipeline();
        CameraShenanigans cam = new CameraShenanigans(hardwareMap, dashboard, pipe);

        ElapsedTime hzTimer = new ElapsedTime();
        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        cam.streamDash();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            previous1.copy(current1);
            current1.copy(gamepad1);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            ///*

            double heading = pipe.getSampleXValue() / 10;

            if (test) {
                heading = swerve.getJustHeadingInDegrees();
                headingTarget = Maths.crownJewel(lastY)[0];
                gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            }
            else {
                lastY = pipe.getSampleYValue();
                headingTarget = 0;
                gamepad1.setLedColor(0,1,0,Gamepad.LED_DURATION_CONTINUOUS);
            }

            if (current1.left_bumper && !previous1.left_bumper) {
                headingTarget = heading;
                adjustToggle.toTrue();
            }

            if (gamepad1.options) {
                swerve.resetIMU();
            }


            swerve.drive(0, 0, headingPID.pidAngleOut(headingTarget, heading));


            telemetry.addData("hz", hzTimer.milliseconds());
            telemetry.addData("peice",Math.toDegrees(Maths.peicewiseAtan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)) - 90);
            telemetry.addData("y",pipe.getSampleYValue());
            telemetry.addData("x",pipe.getSampleXValue());



            //*/

            //pp.changeThresh(t1, t2, t3, t4, t5, t6);

            hzTimer.reset();
            telemetry.update();
        }
        cam.stopStreaming();
        pipe.releaseMats();
        pp.releaseMats();
    }
}

