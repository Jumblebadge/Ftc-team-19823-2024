package org.firstinspires.ftc.teamcode.utility;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.vision.HSVDetectElement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraShenanigans {

    private AprilTagProcessor aprilTag;
    private VisionProcessor HSVDetection;
    private VisionPortal visionPortal;
    private Telemetry telemetry;
    private FtcDashboard dashboard;

    List<AprilTagDetection> currentDetections;
    public double[] tag1Values = {0, 18, 0};
    public double[] tag2Values = {0, 18, 0};
    public double[] tag3Values = {0, 18, 0};
    public double[] tag4Values = {0, 18, 0};
    public double[] tag5Values = {0, 18, 0};
    public double[] tag6Values = {0, 18, 0};

    public CameraShenanigans(Telemetry telemetry, HardwareMap hardwareMap, FtcDashboard dashboard) {
        this.telemetry = telemetry;
        this.dashboard = dashboard;
        // Create the AprilTag processor.
        HSVDetection = new HSVDetectElement();
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(800, 448));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessors(aprilTag, HSVDetection);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public void streamCamera(boolean enabled) {
        if (enabled) dashboard.startCameraStream((CameraStreamSource) aprilTag, 0);
        else dashboard.stopCameraStream();
    }

    public void enableAprilTag(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTag, enabled);
    }

    public void enableHSVDetection(boolean enabled) {
        visionPortal.setProcessorEnabled(HSVDetection, enabled);
    }

    public void update() {
        currentDetections = aprilTag.getDetections();
    }

    public boolean seen() {
        return !currentDetections.isEmpty();
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 1) {tag1Values[0] = detection.ftcPose.x; tag1Values[1] = detection.ftcPose.y; tag1Values[2] = detection.ftcPose.z; }
                if (detection.id == 2) {tag2Values[0] = detection.ftcPose.x; tag2Values[1] = detection.ftcPose.y; tag2Values[2] = detection.ftcPose.z; }
                if (detection.id == 3) {tag3Values[0] = detection.ftcPose.x; tag3Values[1] = detection.ftcPose.y; tag3Values[2] = detection.ftcPose.z; }
                if (detection.id == 4) {tag4Values[0] = detection.ftcPose.x; tag4Values[1] = detection.ftcPose.y; tag4Values[2] = detection.ftcPose.z; }telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                if (detection.id == 5) {tag5Values[0] = detection.ftcPose.x; tag5Values[1] = detection.ftcPose.y; tag5Values[2] = detection.ftcPose.z; }
                if (detection.id == 6) {tag6Values[0] = detection.ftcPose.x; tag6Values[1] = detection.ftcPose.y; tag6Values[2] = detection.ftcPose.z; }telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}
