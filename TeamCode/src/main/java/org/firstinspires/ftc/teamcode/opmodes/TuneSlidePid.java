package org.firstinspires.ftc.teamcode.opmodes;

//Import EVERYTHING we need

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.MedianFilter;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

@Config
@TeleOp(name="tune slide pid", group="Linear Opmode")
public class TuneSlidePid extends LinearOpMode {

    public static double maxVel = 1, maxAccel = 1, maxJerk = 1, Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 1;
    public static double reference = 0, offset = 0;
    private double hz = 0,nanoTime = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        PID pid = new PID(0, 0, 0, 0, 1);

        DcMotorExW pivotLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Llift"));
        DcMotorExW pivotRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Rlift"));
        pivotRight.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotRight.setPowerThresholds(0.05,0.05);
        pivotLeft.setPowerThresholds(0.05,0.05);
        MotorGroup pivotMotors = new MotorGroup(pivotLeft, pivotRight);
        MedianFilter filter = new MedianFilter(7);

        RunMotionProfile profile = new RunMotionProfile(1, 1, 1, new ConstantsForPID(0.3, 0.0015, 0.32, 0.16, 3, 0));

        AnalogInput pivotEncoder = hardwareMap.get(AnalogInput.class, "ma3");

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            double state = pivotEncoder.getVoltage() * 74.16 + offset;
            state = -AngleUnit.normalizeDegrees(state);
            state = filter.getFilteredValue(state);

            pivotMotors.setPowers(profile.profiledPivotMovement(reference, state));

            //motor.setPower(pid.pidOut(reference - motor.getCurrentPosition()));

            profile.setMotionConstraints(maxVel, maxAccel, maxJerk);


            double nano = System.nanoTime();
            hz = (1000000000 / (nano - nanoTime));
            telemetry.addData("hz", hz);
            nanoTime = nano;

            telemetry.addData("Reference", reference);
            telemetry.addData("profle",profile.getMotionTarget());
            telemetry.addData("ma3",state);
            telemetry.update();
        }
    }
}
