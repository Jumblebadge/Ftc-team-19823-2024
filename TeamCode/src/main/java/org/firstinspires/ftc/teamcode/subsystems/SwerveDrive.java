package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.maths.swerveKinematics;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;

public class SwerveDrive {

    final private IMU imu;
    final private DcMotorExW mod1m1,mod1m2,mod2m1,mod2m2;
    final private AnalogInput module1Encoder, module2Encoder;
    final private Telemetry telemetry;
    private final boolean efficientTurnActive;
    private double module1Offset = 78, module2Offset = 45;
    private final PID module1PID = new PID(0.1,0.00188,0.1,0.05, 1);
    private final PID module2PID = new PID(0.1,0.00188,0.1,0.05, 1);
    private final swerveKinematics swavemath = new swerveKinematics();

    double mod1reference;
    double mod2reference;

    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap, boolean eff){
        mod1m1 = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"mod1m1"));
        mod1m2 = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"mod1m2"));
        mod2m1 = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"mod2m1"));
        mod2m2 = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"mod2m2"));
        module1Encoder = hardwareMap.get(AnalogInput.class, "mod1E");
        module2Encoder = hardwareMap.get(AnalogInput.class, "mod2E");

        mod1m1.setPowerThresholds(0.05,0.01);
        mod1m2.setPowerThresholds(0.05,0.01);
        mod2m1.setPowerThresholds(0.05,0.01);
        mod2m2.setPowerThresholds(0.05,0.01);

        mod2m1.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMU(hardwareMap);

        this.telemetry = telemetry;
        this.efficientTurnActive = eff;
    }

    public void drive(double x, double y, double rot){

        //Turn our MA3 absolute encoder signals from volts to degrees
        double mod1P = module1Encoder.getVoltage() * 74.16;
        double mod2P = module2Encoder.getVoltage() * 74.16;

        //zero the encoder values
        mod2P -= module2Offset;
        mod1P -= module1Offset;

        //Update heading of robot
        imu.updateHeading(2);

        double heading = -imu.getHeadingInDegrees();

        //Retrieve the angle and power for each module
        double[] output = swavemath.calculate(y,x,rot, heading,true);
        double mod1power = output[0];
        double mod2power = output[1];

        //keep previous module heading if joystick not being used
        if (y != 0 || x != 0 || rot != 0){
            mod1reference = -output[2];
            mod2reference = -output[3];
        }

        //Anglewrap all the angles so that the module turns both ways
        mod1P = Maths.angleWrap(mod1P);
        mod2P = Maths.angleWrap(mod2P);

        mod1reference = Maths.angleWrap(mod1reference);
        mod2reference = Maths.angleWrap(mod2reference);

        //Make sure that a module never turns more than 90 degrees
        double[] mod1efvalues = Maths.efficientTurn(mod1reference,mod1P,mod1power);

        double[] mod2efvalues = Maths.efficientTurn(mod2reference,mod2P,mod2power);

        if (efficientTurnActive) {
            mod1reference = mod1efvalues[0];
            mod1power = mod1efvalues[1];
            mod2reference = mod2efvalues[0];
            mod2power = mod2efvalues[1];
        }

        //change coax values into diffy values from pid and power
        double[] mod1values = Maths.diffyConvert(module1PID.pidAngleOut(mod1reference, mod1P),mod1power);
        mod1m1.setPower(mod1values[0]);
        mod1m2.setPower(mod1values[1]);
        double[] mod2values = Maths.diffyConvert(module2PID.pidAngleOut(mod2reference, mod2P),mod2power);
        mod2m1.setPower(mod2values[0]);
        mod2m2.setPower(mod2values[1]);

        telemetry.addData("mod1reference",mod1reference);
        telemetry.addData("mod2reference",mod2reference);

        telemetry.addData("mod1P",mod1P);
        telemetry.addData("mod2P",mod2P);
    }

    public void resetIMU() {
        imu.resetIMU();
    }

    //tune module PIDs
    public void setPIDCoeffs(double Kp, double Kd,double Ki, double Kf, double limit){
        module1PID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    //tunable module zeroing
    public void setModuleAdjustments(double module1Adjust, double module2Adjust){
        this.module1Offset = module1Adjust;
        this.module2Offset = module2Adjust;
    }

    public double getHeading() {
        return imu.getHeadingInDegrees();
    }
}
