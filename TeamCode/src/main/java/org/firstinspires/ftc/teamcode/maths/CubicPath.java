package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

/**
 * Path comprised of 3 bezier curves. C1 continuous. Using 3 seperate curves allows for local control over the curve.
 * @see Bezier
 */
public class CubicPath {

    public Bezier[] beziers = new Bezier[3];
    public double guessT = 0, arcLength = 0;
    private double totalArcLength;
    private Vector2d[] controlPoints;
    public Vector2d[] temp;
    double[] arcLengths = new double[3];

    /**
     * Converts raw x and y values into 3 seperate bezier curves. The end of each bezier is connected to the start of the next,
     * and the two control points opposite the joining points are exact mirrors of the point before.
     * This ensures C1 continuity throughout the entire bezier curve.
     * @param rawControlPoints list of doubles representing the x and y coordinates of all 8 points used to control the bezier.
     */
    public CubicPath(double[] rawControlPoints) {
        controlPoints = Maths.pointListToVectorList(rawControlPoints);
        temp = controlPoints;
        beziers[0] = new Bezier(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3]);
        beziers[1] = new Bezier(controlPoints[3], controlPoints[3].times(2).minus(controlPoints[2]), controlPoints[4], controlPoints[5]);
        beziers[2] = new Bezier(controlPoints[5], controlPoints[5].times(2).minus(controlPoints[4]), controlPoints[6], controlPoints[7]);

        calculateTotalArcLength();
    }

    /**
     * Converts vector values into 3 seperate bezier curves. The end of each bezier is connected to the start of the next,
     * and the two control points opposite the joining points are exact mirrors of the point before.
     * This ensures C1 continuity throughout the entire bezier curve.
     * @param controlPoints list of vectors which are all 8 points used to control the bezier.
     */
    public CubicPath(Vector2d[] controlPoints) {
        this.controlPoints = controlPoints;
        beziers[0] = new Bezier(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3]);
        beziers[1] = new Bezier(controlPoints[3], controlPoints[3].times(2).minus(controlPoints[2]), controlPoints[4], controlPoints[5]);
        beziers[2] = new Bezier(controlPoints[5], controlPoints[5].times(2).minus(controlPoints[4]), controlPoints[6], controlPoints[7]);

        calculateTotalArcLength();
    }

    public void setControlPointCoordinates(double[] controlPointCoordinates) {
        controlPoints = Maths.pointListToVectorList(controlPointCoordinates);
        beziers[0].setControlPoints(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3]);
        beziers[1].setControlPoints(controlPoints[3], controlPoints[3].times(2).minus(controlPoints[2]), controlPoints[4], controlPoints[5]);
        beziers[2].setControlPoints(controlPoints[5], controlPoints[5].times(2).minus(controlPoints[4]), controlPoints[6], controlPoints[7]);

        guessT = 0;
        arcLength = 0;
        calculateTotalArcLength();
    }

    public Vector2d[] getControlPointList() {
        Vector2d[] out = new Vector2d[8];
        int i = 0;
        for (Vector2d v : controlPoints) {
            out[i] = v.times(1);
            i++;
        }
        return out;
    }

    /**
     * find spline's total arc length, and save the arc length
     */
    public void calculateTotalArcLength() {
        totalArcLength = 0;
        for (int i = 0; i < beziers.length; i++) {
            totalArcLength += beziers[i].getTotalArcLength();
            arcLengths[i] = beziers[i].getTotalArcLength();
        }
    }

    public double getTotalArcLength() {
        return totalArcLength;
    }

    /**
     * return the point along the spline based off given T value
     * @param T [0, 3)
     * @return Point at given T value from the correct bezier
     */
    public Vector2d getPoint(double T) {
        // bounds may be off? check
        //if (T < 0) { T = 0; }
        //if (T >= 3) { T = 2.9999; }
        return beziers[(int) T].getPoint(T - Math.floor(T));
    }

    //DOES NOT WORK
    //TODO FIX
    public Vector2d getControlPoint(int index) {
        return controlPoints[index];
    }

    /**
     * Returns the normalized tangent vector to the path at specified point
     * @param T value at which to calculate tangent vector
     * @return tangent to the path at specified point
     */
    public Vector2d getNormalizedTangent(double T) { return beziers[(int) T].getNormalizedTangent(T - Math.floor(T)); }

    /**
     * Returns the normalized normal vector to the path at specified point
     * @param T value at which to calculate tangent vector
     * @return normal to the path at specified point
     */
    public Vector2d getNormalizedNormal(double T) { return beziers[(int) T].getNormalizedNormal(T - Math.floor(T)); }

    /**
     * given an arc length along the curve, find which bezier curve that distance lies on.
     * @param arcLength along curve
     * @return index of bezier curve at which the arcLength value lies
     */
    public int whichBezierFromDistance(double arcLength) {
        if (arcLength <= arcLengths[0]) {
            return 0;
        }
        else if (arcLength <= arcLengths[0] + arcLengths[1]) {
            return 1;
        }
        else if (/*arcLengths[0] + arcLengths[1] <= arcLength && */arcLength <= arcLengths[0] + arcLengths[1] + arcLengths[2]) {
            return 2;
        }
        //throws error if the given arclength is not within any of the beziers
        return -(int) arcLength;
        //return 2;
    }

    /**
     * finds the T value along the path at a specified arc length
     * @param arcLength along path at which to find T value
     * @return T value along entire path at which the given arc length lies
     */
    public double distanceToT(double arcLength) {
        int bezier = whichBezierFromDistance(arcLength);
        double minus = 0;
        if (bezier == 1) { minus = arcLengths[0]; }
        if (bezier == 2) { minus = arcLengths[0] + arcLengths[1]; }
        return (beziers[bezier].distanceToT(arcLength - minus)) + bezier;
    }

    /**
     * Find the closest point on the entire path to a point, which can be anywhere in space. This is achieved through Newton's method,
     * essentially attempting to minimize distance. Solution converges after ~8 loops, but 10 can be done just to be sure of the accuracy
     * of the projection.
     * @param point in cartesian space which we are iterating from. NOT ON the path. will usually be the robot's current position
     * @return closest point ON the path to the inputted point
     */
    public Vector2d findClosestPointOnPath(Vector2d point) {
        for (int i = 0; i < 10; i++) {
            Vector2d guess = getPoint(guessT);
            Vector2d robotVector = point.minus(guess);
            Vector2d normalizedTangent = getNormalizedTangent(guessT);
            double dotProduct = normalizedTangent.dot(robotVector);
            //Dot product is applied to an error mapping function [-1,1], because otherwise newton's method results in oscillations
            //This only works with fast loop times, and this was an arbitrary number. Not the best solution
            dotProduct = Maths.tanhErrorMap(dotProduct);
            arcLength += dotProduct;
            arcLength = Range.clip(arcLength, 0.0001, getTotalArcLength() - 0.0001);
            guessT = distanceToT(arcLength);
        }
        return getPoint(guessT);
    }

}
