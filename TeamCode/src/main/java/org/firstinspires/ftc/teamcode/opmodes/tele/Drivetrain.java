package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;

@Config
@TeleOp(name="Drivetrain", group="Linear Opmode")
public class Drivetrain extends LinearOpMode {

    public static double module1Offset = 37, module2Offset = -6;
    private double rotation, heading;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);

        ElapsedTime hztimer = new ElapsedTime();

        PID headingPID = new PID(0.1,0.00188,0,0.05,1);
        ButtonDetector headingPIDtoggle  = new ButtonDetector();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {

            Pose2d pose = swerve.getPose();
            previous1.copy(current1);
            current1.copy(gamepad1);

            swerve.setModuleAdjustments(module1Offset, module2Offset);

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            if (headingPIDtoggle.toggle(gamepad1.a)) {
                rotation = headingPID.pidAngleOut(heading, swerve.getHeadingInDegrees());
            }
            else { rotation = -gamepad1.right_stick_x; }

            if (current1.x && !previous1.x) {
                heading = swerve.getHeadingInDegrees();
            }

            if (current1.dpad_up && !previous1.dpad_up) {
                heading = 0;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_right && !previous1.dpad_right) {
                heading = 90;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_down && !previous1.dpad_down) {
                heading = 180;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_left && !previous1.dpad_left) {
                heading = -90;
                headingPIDtoggle.toTrue();
            }


            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation);

            if (gamepad1.b) {
                //swerve.resetIMU();
            }


            telemetry.addData("millis",hztimer.milliseconds());
            telemetry.addData("pose", pose.toString());
            telemetry.addData("heading",pose.getHeading());
            telemetry.addData("heading2",swerve.getHeadingInDegrees());
            hztimer.reset();
            telemetry.update();
        }
    }
}
