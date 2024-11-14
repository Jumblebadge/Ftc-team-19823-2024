package org.firstinspires.ftc.teamcode.opmodes;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;

@Config
@TeleOp(name="tune pid", group="Linear Opmode")
public class TunePID extends LinearOpMode {

    public static double Kp = 0.1, Kd = 0.00188, Ki = 0.1, Kf = 0.05, Kl = 1;
    public static double reference = 0, offset = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        PID pid = new PID(0, 0, 0, 0, 1);

        DcMotorExW motor = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "motor"));
        DcMotorExW motor2 = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "motor2"));
        AnalogInput ma3 = hardwareMap.get(AnalogInput.class, "ma3");

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            double[] mod1values = Maths.diffyConvert(pid.pidAngleOut(reference, ma3.getVoltage() * 74.16 + offset),gamepad1.right_stick_y);
            motor.setPower(mod1values[0]);
            motor2.setPower(mod1values[1]);

            //motor.setPower(pid.pidOut(reference - motor.getCurrentPosition()));

            pid.setPIDgains(Kp, Kd, Ki, Kf, Kl);

            telemetry.addData("Reference", reference);
            telemetry.addData("state", AngleUnit.normalizeDegrees(AngleUnit.normalizeDegrees(ma3.getVoltage() * 74.16) + offset));
            telemetry.addData("satse1", ma3.getVoltage() * 74.16);
            telemetry.addData("Kp", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[1]);
            telemetry.addData("Kd", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[2]);
            telemetry.addData("Ki", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[3]);
            telemetry.addData("Kf", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[4]);
            telemetry.addData("timer", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[5]);
            telemetry.addData("derivative", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[6]);
            telemetry.addData("error", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[7]);
            //telemetry.addData("satte", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
