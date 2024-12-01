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
import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.MedianFilter;
import org.firstinspires.ftc.teamcode.subsystems.ThreeAxisClaw;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

@Config
@TeleOp(name="tst", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static double motionTarget = 0;
    private double nanoTime, average, count;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        DcMotorExW motor = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "motor"));

        RunMotionProfile profile = new RunMotionProfile(3000,3000,3000,new ConstantsForPID(0.6,0,0.2,0.1,3,0));

        //class to swerve the swerve
        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {


            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();


            profile.profiledMovement(motionTarget, motor.getCurrentPosition());
            telemetry.addData("motion taraget", profile.getMotionTarget());
            telemetry.addData("state", motor.getCurrentPosition());
            telemetry.addData("target",motionTarget);

            telemetry.addData("hz", 1000000000 / (System.nanoTime() - nanoTime));
            average += 1000000000 / (System.nanoTime() - nanoTime);
            count++;
            nanoTime = System.nanoTime();
            telemetry.addData("avearag",average / count);


            hztimer.reset();
            telemetry.update();
        }
    }
}
