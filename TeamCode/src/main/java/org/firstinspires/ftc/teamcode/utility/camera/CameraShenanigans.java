package org.firstinspires.ftc.teamcode.utility.camera;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class CameraShenanigans {

    private FtcDashboard dashboard;
    private WebcamName webcam;
    private VisionProcessor processor;

    private VisionPortal portal;

    public CameraShenanigans(HardwareMap hardwareMap, FtcDashboard dashboard, VisionProcessor processor) {
        this.dashboard = dashboard;
        this.processor = processor;

        webcam = hardwareMap.get(WebcamName.class, "webcam");

        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    public void enableProcessor(boolean enabled) {
        portal.setProcessorEnabled(processor, enabled);
    }

    public void streamToDash(boolean enabled) {
        if (enabled) dashboard.startCameraStream((CameraStreamSource) processor, 0);
        else dashboard.stopCameraStream();
    }


}
