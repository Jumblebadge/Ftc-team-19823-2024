package org.firstinspires.ftc.teamcode.utility.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.util.Range;

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
public class BlueECircle extends OpenCvPipeline {

    private Mat hsv = new Mat();
    private Mat thresh = new Mat();
    private Mat edges = new Mat();

    private double maxArea;
    private MatOfPoint largestContour;

    private ArrayList<MatOfPoint> contours;

    Point centroid;
    float[] radius = new float[1];


    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));

    Paint paint;

    public BlueECircle() {
        contours = new ArrayList<MatOfPoint>();

        paint = new Paint();
        paint.setColor(Color.WHITE); // you may want to change this
        paint.setAntiAlias(true);
        paint.setStrokeWidth(10); // or this
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStrokeJoin(Paint.Join.ROUND);
        paint.setStyle(Paint.Style.STROKE);
    }


    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, new Scalar(96, 110, 0), new Scalar(137, 255, 255), thresh);

        Imgproc.erode(thresh, thresh, kernel, new Point(-1, -1), 6);
        Imgproc.dilate(thresh, thresh, kernel, new Point(-1, -1), 6);

        Imgproc.GaussianBlur(thresh, thresh, new Size(11, 11), 0);

        //i am worried about these values
        Imgproc.Canny(thresh, edges, 0, 255);

        //here comes the fun part!
        // i am assigning hsv here because i do not need the heirarchy variable
        Imgproc.findContours(edges, contours, hsv, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        maxArea = 0;

        largestContour = null;

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

            // find the biggest contour with 4 or 6 edges. A rectangle in any orientation from our viewpoint must have this many vertices.
            if (numEdges >= 4 && numEdges <= 6 && area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
            else contour.release();
            approximation.release();
            contour2f.release();
        }
        contours.clear();

        if (detected()) {
            Moments moment = Imgproc.moments(largestContour);
            if (moment.get_m00() != 0) {
                centroid = new Point(moment.get_m10() / moment.get_m00(), moment.get_m01() / moment.get_m00());
            }
            else centroid = null;
            Point temp = new Point();
            MatOfPoint2f point = new MatOfPoint2f(largestContour.toArray());

            //We find the min enclosing circle around the largest contour.
            //This method provides stable readings even with the sample being oriented differently.
            Imgproc.minEnclosingCircle(point, temp, radius);
        }

        if (detected()) largestContour.release();
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
        return largestContour != null;
    }

    public void releaseMats() {
        hsv.release();
        thresh.release();
        edges.release();

        if (detected()) largestContour.release();


        kernel.release();
    }



}
