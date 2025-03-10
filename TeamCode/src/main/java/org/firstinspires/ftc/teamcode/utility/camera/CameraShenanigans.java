package org.firstinspires.ftc.teamcode.utility.camera;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.utility.wrappers.ServoImplExW;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraShenanigans {

    private FtcDashboard dashboard;
    private OpenCvWebcam webcam;

    private ServoImplExW light;

    private VisionPortal portal;

    public CameraShenanigans(HardwareMap hardwareMap, FtcDashboard dashboard, OpenCvPipeline pipeline) {
        this.dashboard = dashboard;

        light = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "light"));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                resumeStreaming();
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public void stopStreaming() {
        webcam.stopStreaming();
    }

    public void resumeStreaming() {
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    public void streamDash() {
        dashboard.startCameraStream(webcam, 30);
    }

    public void stopDashStream() {
        dashboard.stopCameraStream();
    }

    public void pauseViewPort() {
        webcam.pauseViewport();
    }

    public void resumeViewPort() {
        webcam.resumeViewport();
    }

    public void setLightPower(double power) {
        light.setPosition(power);
    }

}
