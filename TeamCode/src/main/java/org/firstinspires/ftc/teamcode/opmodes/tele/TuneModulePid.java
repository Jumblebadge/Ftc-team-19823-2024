package org.firstinspires.ftc.teamcode.opmodes.tele;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;

import org.firstinspires.ftc.teamcode.maths.MedianFilter;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

@Config
@Disabled
@TeleOp(name="tune modulue pid", group="Linear Opmode")
public class TuneModulePid extends LinearOpMode {

    public static double Kp = 0.1, Kd = 0.00188, Ki = 0.1, Kf = 0.05, Kl = 1;
    public static double reference = 0, offset = 0;
    public static double mod1 = 0, mod2 = 0;
    private int count = 0;
    private double hz = 0,nanoTime = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SwerveDrive drive = new SwerveDrive(telemetry, hardwareMap);

        //DcMotorExW motor = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "mod1m1"));
        //DcMotorExW motor2 = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "mod1m2"));
        //AnalogInput ma3 = hardwareMap.get(AnalogInput.class, "mod1E");

        //PID pid = new PID();


        MedianFilter filter = new MedianFilter(7);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            //double[] mod1values = Maths.diffyConvert(pid.pidAngleOut(reference, ma3.getVoltage() * 74.16 + offset),gamepad1.right_stick_y);
            //motor.setPower(mod1values[0]);
            //motor2.setPower(mod1values[1]);

            //motor.setPower(pid.pidOut(reference - motor.getCurrentPosition()));

            drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            drive.setModuleAdjustments(mod1, mod2);

            double nano = System.nanoTime();
            hz = (1000000000 / (nano - nanoTime));
            count++;
            telemetry.addData("hz", hz);
            nanoTime = nano;

            //double test = filter.getFilteredValue(AngleUnit.normalizeDegrees(ma3.getVoltage() * 74.16));
            //telemetry.addData("Reference", reference);
            //telemetry.addData("state", AngleUnit.normalizeDegrees(AngleUnit.normalizeDegrees(ma3.getVoltage() * 74.16) + offset));
            //telemetry.addData("satse1", ma3.getVoltage() * 74.16);
            //telemetry.addData("state 2", test);
            //telemetry.addData("error of measurement", AngleUnit.normalizeDegrees((ma3.getVoltage() * 74.16) + offset) - test);
            //telemetry.addData("Kp", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[1]);
            //telemetry.addData("Kd", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[2]);
            //telemetry.addData("Ki", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[3]);
            //telemetry.addData("Kf", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[4]);
            //telemetry.addData("timer", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[5]);
            //telemetry.addData("derivative", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[6]);
            //telemetry.addData("error", pid.inDepthOutput(reference, ma3.getVoltage() * 74.16 + offset)[7]);
            //telemetry.addData("satte", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
