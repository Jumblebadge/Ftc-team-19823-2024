package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMU {

    private final BNO055IMU imu;
    private double imuOffset = 0, count = 0;
    private Orientation angle;

    public IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        updateHeading(0);
    }

    public double getHeadingInDegrees() {
        return AngleUnit.normalizeDegrees(angle.firstAngle * -1 - imuOffset);
    }

    public double getHeadingInRadians() {
        return AngleUnit.normalizeRadians(AngleUnit.DEGREES.toRadians(angle.firstAngle) + Math.toRadians(imuOffset));
    }

    public void updateHeading(int interval) {
        if (count >= interval - 1) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            count = 0;
        }
        else count++;
    }

    public void setImuOffset(double offset) {
        imuOffset += offset;
    }

    public void resetIMU() {
        setImuOffset(getHeadingInDegrees());
    }
}
