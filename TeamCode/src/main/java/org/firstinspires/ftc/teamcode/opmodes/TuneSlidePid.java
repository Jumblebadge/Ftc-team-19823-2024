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

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.subsystems.PivotingSlide;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

@Config
@TeleOp(name="tune slide pid", group="Linear Opmode")
public class TuneSlidePid extends LinearOpMode {

    public static double maxVel = 1, maxAccel = 1, maxJerk = 1, Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 1;
    public static double slideReference = 0, pivotReference = 90;
    private double hz = 0,nanoTime = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        PivotingSlide slide = new PivotingSlide(hardwareMap);

        DcMotorExW liftLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Lpivot"));
        DcMotorExW liftRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Rpivot"));

        AnalogInput ma3 = hardwareMap.get(AnalogInput.class, "pivotEncoder");

        MotorGroup slideMotors = new MotorGroup(liftLeft, liftRight);

        RunMotionProfile profile = new RunMotionProfile(3000,3000,3000,new ConstantsForPID(0.6,0,0.2,0.1,3,0));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        slide.resetEncoders();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            slideMotors.setPowers(profile.profiledPivotMovement(slideReference, -ma3.getVoltage() * 74.16  + 164));
            profile.setPidConstants(new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, 0));

            //slide.update();
            //slide.moveSlideTo(slideReference);
            //slide.movePivotTo(pivotReference);




            double nano = System.nanoTime();
            hz = (1000000000 / (nano - nanoTime));
            telemetry.addData("hz", hz);
            nanoTime = nano;

            telemetry.addData("Reference", slideReference);
            //telemetry.addData("motion",slide.getMotionTarget());
            //telemetry.addData("state",slide.getSlidePosition());
            //telemetry.addData("cable",slide.getCableDifference());
            telemetry.addData("motion",profile.getMotionTarget());
            telemetry.addData("state",-ma3.getVoltage() * 74.16  + 164);
            telemetry.update();
        }
    }
}
