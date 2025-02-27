package org.firstinspires.ftc.teamcode.utility.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.maths.Maths;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/**
 * DO NOT FORGET TO CALL RELEASE MATS!
 */
public class YellowECircle extends OpenCvPipeline {

    private Mat ycrcb = new Mat();
    private Mat thresh = new Mat();
    private Mat edges = new Mat();

    private MatOfPoint closestContour;

    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private Point centroid;
    float[] radius = new float[1];

    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));

    private final Paint paint = new Paint();;

    private Telemetry telemetry;

    public YellowECircle(Telemetry telemetry ) {
        paint.setColor(Color.WHITE);
        paint.setAntiAlias(true);
        paint.setStrokeWidth(10);
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStrokeJoin(Paint.Join.ROUND);
        paint.setStyle(Paint.Style.STROKE);

        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycrcb, new Scalar(0, 120, 0), new Scalar(255, 225, 90), thresh);

        Imgproc.erode(thresh, thresh, kernel, new Point(-1, -1), 6);
        Imgproc.dilate(thresh, thresh, kernel, new Point(-1, -1), 6);

        Imgproc.GaussianBlur(thresh, thresh, new Size(11, 11), 0);

        //i am worried about these values
        Imgproc.Canny(thresh, edges, 50, 150);

        //here comes the fun part!
        // i am assigning hsv here because i do not need the heirarchy variable
        Imgproc.findContours(edges, contours, ycrcb, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        double leastX = 100000;

        closestContour = null;

        centroid = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 1000) {
                contour.release();
                continue;
            }

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double perimeter = Imgproc.arcLength(contour2f, true);

            MatOfPoint2f approximation = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approximation, perimeter * 0.02, true);

            int numEdges = approximation.toArray().length;

            if (numEdges < 4 || numEdges > 6) {
                contour.release();
                continue;
            }

            Moments moment = Imgproc.moments(contour);
            if (moment.get_m00() != 0) {
                centroid = new Point(moment.get_m10() / moment.get_m00(), moment.get_m01() / moment.get_m00());
                if (Math.abs(320 - centroid.x) < leastX) {
                    closestContour = contour;
                    leastX = 320 - centroid.x;
                }
                else contour.release();
            }
            else {
                centroid = null;
                contour.release();
            }
            telemetry.addData("leastX", leastX);

            approximation.release();
            contour2f.release();
        }
        contours.clear();

        if (detected()) {
            Point temp = new Point();
            MatOfPoint2f point = new MatOfPoint2f(closestContour.toArray());

            //We find the min enclosing circle around the largest contour.
            //This method provides stable readings even with the sample being oriented differently.
            Imgproc.minEnclosingCircle(point, temp, radius);
            closestContour.release();
        }
        return edges;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (detected()) {
            paint.setColor(Color.CYAN);
            canvas.drawCircle((float) centroid.x * scaleBmpPxToCanvasPx, (float) centroid.y * scaleBmpPxToCanvasPx, radius[0] * scaleBmpPxToCanvasPx, paint);
            paint.setColor(Color.GREEN);
            canvas.drawCircle((float) centroid.x * scaleBmpPxToCanvasPx, (float) centroid.y * scaleBmpPxToCanvasPx, 5, paint);
        }
    }

    public float getCircleRadius() {
        if (!detected()) return -1;
        return radius[0];
    }

    /**
     * Gives distance in inches from the camera to the detected sample. This is assuming a top down view, so height of the camera does not contribute to distance
     * It is simply the length of a straight line from the camera's (x, y) position.
     * This is calculated using polynomial regression to simulate the camera distortion.
     * This works because we are aligning our robot to the sample already, so it must only undistort in the Y direction.
     * @return length of a straight line from the camera's (x, y) position to sample.
     */
    public double getSampleYValue() {
        if (!detected()) return -1;
        double radius = getCircleRadius();
        radius = Range.clip(radius, 50, 100);

        return Maths.calculateQuadratic(0.00662535, -1.36576, 77.33902, radius);
    }

    public Point getCentroid() {
        if (!detected()) return null;
        return centroid;
    }

    /**
     * @return X distance in pixels from center of image to the center of largest detected sample.
     */
    public double getSampleXValue() {
        if (centroid == null) return 0;
        return centroid.x - 320;
    }

    public boolean detected() {
        return closestContour != null;
    }

    public void releaseMats() {
        ycrcb.release();
        thresh.release();
        edges.release();

        if (detected()) closestContour.release();


        kernel.release();
    }



}
