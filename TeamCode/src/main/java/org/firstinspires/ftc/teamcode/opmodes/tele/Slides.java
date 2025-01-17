package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.ThreeAxisClaw;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;

@Config
@TeleOp(name="Slides", group="Linear Opmode")
public class Slides extends LinearOpMode {

    public static double targetClaw = 0, targetWrist = 0, targetRotator = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");


        ElapsedTime hztimer = new ElapsedTime();

        ThreeAxisClaw claw = new ThreeAxisClaw(hardwareMap);
        //PivotingSlide slide = new PivotingSlide(hardwareMap);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {


            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            claw.setClawPosition(targetClaw);
            claw.setWristPosition(targetWrist);
            claw.setRotatorPosition(targetRotator);

            telemetry.addData("millis",hztimer.milliseconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
