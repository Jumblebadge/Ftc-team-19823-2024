package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;
import org.firstinspires.ftc.teamcode.utility.camera.BrushColor;
import org.firstinspires.ftc.teamcode.utility.wrappers.CRServoImplExW;
import org.firstinspires.ftc.teamcode.utility.wrappers.ServoImplExW;

import java.util.List;

@Config
@TeleOp(name="test", group="Linear Opmode")
public class Test extends LinearOpMode {

    public static boolean on = false;

    ElapsedTimeW timer = new ElapsedTimeW();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        Intake intake = new Intake(hardwareMap);
        BrushColor color = new BrushColor(hardwareMap);

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            previous1.copy(current1);
            current1.copy(gamepad1);
            previous2.copy(current2);
            current2.copy(gamepad2);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            intake.setWristDown();
            intake.setSpinIn();
            intake.setRotatorTo0();

            if (!(intake.getColor() == BrushColor.ColorDetection.YELLOW || intake.getColor() == BrushColor.ColorDetection.BLUE)) {
                timer.reset();
            }
            on = timer.seconds() > 0.5;


            if (intake.getColor() == BrushColor.ColorDetection.RED) {
            intake.setLatchOpen();
            }
            else intake.setLatchClose();


            telemetry.addData("color", color.getDetection());
            telemetry.addData("0", color.getPin0State());
            telemetry.addData("1", color.getPin1State());
            telemetry.addData("on", on);

            telemetry.update();
        }
    }
}