package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utility.GoBildaPinpointDriver;

public class PinPoint {

    private GoBildaPinpointDriver pinpoint;
    private double offset;

    public PinPoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(-87.27, -28.5);
    }

    public Pose2d getPoseIn(DistanceUnit distanceUnit, AngleUnit angleUnit) {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        if (angleUnit == AngleUnit.RADIANS) {
            return new Pose2d(pose.getX(distanceUnit), pose.getY(distanceUnit), AngleUnit.normalizeRadians(pose.getHeading(angleUnit) - offset));
        }
        return new Pose2d(pose.getX(distanceUnit), pose.getY(distanceUnit), AngleUnit.normalizeDegrees(pose.getHeading(angleUnit) - Math.toDegrees(offset)));
    }

    public double getJustHeadingInRadians() {
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        return pinpoint.getHeading() - offset;
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }

    public void setPosition(Pose2d position) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, position.getX(), position.getY(), AngleUnit.DEGREES, position.getHeading()));
    }

    public void resetPoseAndHeading() {
        pinpoint.resetPosAndIMU();
    }

    public void resetHeading() {
        offset = pinpoint.getHeading();
    }
}
