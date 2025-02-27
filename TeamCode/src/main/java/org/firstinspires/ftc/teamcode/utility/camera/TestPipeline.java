package org.firstinspires.ftc.teamcode.utility.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {

    Mat hsv = new Mat();

    Paint paint;

    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;

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
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(hsv, new Scalar(t1, t2, t3), new Scalar(t4, t5, t6), hsv);

        return hsv;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawLine(0,0, 500, 500, paint);
    }

    public void releaseMats() {
        hsv.release();
    }

    public void changeThresh(double t1, double t2, double t3, double t4, double t5, double t6) {
        this.t1 = t1;
        this.t2 = t2;
        this.t3 = t3;
        this.t4 = t4;
        this.t5 = t5;
        this.t6 = t6;

    }
}
