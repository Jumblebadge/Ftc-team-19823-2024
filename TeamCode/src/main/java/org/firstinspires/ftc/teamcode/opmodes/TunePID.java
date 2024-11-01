package org.firstinspires.ftc.teamcode.opmodes;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;

@Config
@TeleOp(name="tune pid", group="Linear Opmode")
public class TunePID extends LinearOpMode {

    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 1;
    public static double reference = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

      //Bulk sensor reads
       LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        PID pid = new PID(0, 0, 0, 0, 1);

        DcMotorExW motor = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "motor"));

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            motor.setPower(pid.pidOut(reference - motor.getCurrentPosition()));

            telemetry.addData("Reference", reference);
            telemetry.addData("satte", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
