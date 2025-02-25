package org.firstinspires.ftc.teamcode.utility.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {

    Mat hsv = new Mat();

    Paint paint;

    public TestPipeline() {
        paint = new Paint();
        paint.setColor(Color.WHITE); // you may want to change this
        paint.setAntiAlias(true);
        paint.setStrokeWidth(10); // or this
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStrokeJoin(Paint.Join.ROUND);
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        return hsv;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawLine(0,0, 500, 500, paint);
    }
}
