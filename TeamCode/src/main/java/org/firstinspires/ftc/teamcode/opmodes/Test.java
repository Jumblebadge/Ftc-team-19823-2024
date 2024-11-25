package org.firstinspires.ftc.teamcode.opmodes;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.MedianFilter;
import org.firstinspires.ftc.teamcode.subsystems.ThreeAxisClaw;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;

@Config
@TeleOp(name="tst", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static double moduleTarget = 0;
    private double nanoTime, theta, max, average, count;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        AnalogInput input = hardwareMap.get(AnalogInput.class, "ma3");

        MedianFilter filter = new MedianFilter(11);

        //class to swerve the swerve
        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {


            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();


            double angle = input.getVoltage() * 72;

            if (Math.abs(input.getVoltage()) > max) max = input.getVoltage();

            telemetry.addData("original", angle);
            angle = AngleUnit.normalizeDegrees(angle);
            telemetry.addData("normalized",angle);
            angle = filter.getFilteredValue(angle);
            telemetry.addData("new", angle);

            telemetry.addData("hz", 1000000000 / (System.nanoTime() - nanoTime));
            average += 1000000000 / (System.nanoTime() - nanoTime);
            count++;
            nanoTime = System.nanoTime();
            telemetry.addData("avearag",average / count);

            telemetry.addData("octant", Math.toDegrees(theta));
            telemetry.addData("theta", Math.toDegrees(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)));
            telemetry.addData("x",gamepad1.right_stick_x);
            telemetry.addData("y",gamepad1.right_stick_y);
            telemetry.addData("max",max);
            telemetry.addData("now",input.getVoltage());
            telemetry.addData("artificial",input.getMaxVoltage());
            hztimer.reset();
            telemetry.update();
        }
    }
}
