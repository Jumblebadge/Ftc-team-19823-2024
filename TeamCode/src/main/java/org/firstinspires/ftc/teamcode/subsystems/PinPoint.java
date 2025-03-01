package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.firstinspires.ftc.teamcode.utility.wrappers.GoBildaPinpointDriver;

public class PinPoint {

    private GoBildaPinpointDriver pinpoint;
    private double offset;
    private Pose2D pose;
    private double heading;

    public PinPoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(76.2, 127);
    }

    //we have to do all of this bs because pinpoint gives faulty readings sometimes
    public Pose2d getPoseIn(DistanceUnit distanceUnit, AngleUnit angleUnit) {
        pinpoint.update();
        Pose2D pose1 = pinpoint.getPosition();
        if (!Double.isNaN(pose1.getX(distanceUnit)) && !Double.isNaN(pose1.getY(distanceUnit))) {
            if (Math.abs(pose1.getHeading(angleUnit)) < 1000 && !Double.isNaN(pose1.getHeading(angleUnit))) {
                pose = new Pose2D(distanceUnit, pose1.getX(distanceUnit), pose1.getY(distanceUnit), angleUnit, pose1.getHeading(angleUnit));
            }
            else pose = new Pose2D(distanceUnit, pose1.getX(distanceUnit), pose1.getY(distanceUnit), angleUnit, pose.getHeading(angleUnit));
        }
        if (angleUnit == AngleUnit.RADIANS) {
            return new Pose2d(pose.getX(distanceUnit), pose.getY(distanceUnit), Maths.angleWrapRadians(pose.getHeading(AngleUnit.RADIANS) - offset));
        }
        return new Pose2d(pose.getX(distanceUnit), pose.getY(distanceUnit), Maths.angleWrapDegrees(pose.getHeading(AngleUnit.DEGREES) - Math.toDegrees(offset)));
    }

    public double getJustHeadingInRadians() {
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        if (Math.abs(pinpoint.getHeading()) < 1000 && !Double.isNaN(pinpoint.getHeading())) {
            heading = pinpoint.getHeading();
        }
        return heading - offset;
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
        if (Math.abs(pinpoint.getHeading()) < 1000 && !Double.isNaN(pinpoint.getHeading())) {
             offset = pinpoint.getHeading();
        }
    }
}
