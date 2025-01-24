package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Bezier curve defined by 4 discrete control points. The curve is parameterized by a T value [0, 1].
 */
public class Bezier {

    private Vector2d A,B,C,D;
    private final int accuracy = 50; // can be edited, 50 is an arbitrary value which i thought was balanced enough.
    private Vector2d[] lookup = new Vector2d[accuracy + 1];
    private double totalArcLength;

    /**
     * A bezier curve must pass through all 4 control points. When created, total arc length of the curve as well as a lookup table of distance -> T
     */
    public Bezier(Vector2d A, Vector2d B, Vector2d C, Vector2d D) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;

        generateLookup();
    }

    public void setControlPoints(Vector2d A, Vector2d B, Vector2d C, Vector2d D) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;

        generateLookup();
    }

    public Vector2d getA() { return this.A; }
    public Vector2d getB() { return this.B; }
    public Vector2d getC() { return this.C; }
    public Vector2d getD() { return this.D; }

    /**
     * Generates a lookup table of ordered pairs (T value, arc length of curve at T value).
     * This allows us to find the distance along the curve for any T value, or the t value for any distance along the curve.
     */
    public void generateLookup() {
        double arc = 0;
        for (int i = 0; i < accuracy; i++) {
            lookup[i] = new Vector2d(i / (double) accuracy, arc);
            arc += Maths.distanceBetween(getPoint(i / (double) accuracy), getPoint((i + 1) / (double) accuracy));
        }
        //by the end, the arc variable has computed the arc length of the entire curve.
        totalArcLength = arc;
        lookup[accuracy] = new Vector2d(1, totalArcLength);
    }

    public double getTotalArcLength() {
        return totalArcLength;
    }

    /**
     * Finds the arc length from 0 to T along the curve using parametrization.
     */
    public double getArcLength(double T) {
        double total = 0;
        for (int i = 0; i < accuracy; i++) {
            if (i / (double) accuracy < T) {
                total += Maths.distanceBetween(getPoint(i / (double) accuracy), getPoint((i + 1) / (double) accuracy));
            }
            else break;
        }
        return total;
    }

    /**
     * Returns a point in cartesian coordinates along the curve
     * @param T value which desired point is at
     * @return (x,y) point on the curve at specified T value
     */
    public Vector2d getPoint(double T) {
        double weightA = (-1 * Math.pow((T),3) + 3 * Math.pow((T),2) - 3 * T + 1);
        double weightB = (3  * Math.pow((T),3) - 6 * Math.pow((T),2) + 3 * T);
        double weightC = (-3 * Math.pow((T),3) + 3 * Math.pow((T),2));
        double weightD = Math.pow((T),3);

        return A.times(weightA).plus(B.times(weightB)).plus(C.times(weightC)).plus(D.times(weightD));
    }

    /**
     * Returns a point in cartesian coordinates along this bezier curve's 1st derivative, which is a bezier curve with 3 control points.
     * @param T value which desired point is at
     * @return (x,y) point on the curve's first derivative at specified T value
     */
    public Vector2d firstDerivative(double T) {
        double weightA = (-3 * Math.pow((T),2) + 6  * T - 3);
        double weightB = (9 *  Math.pow((T),2) - 12 * T + 3);
        double weightC = (-9 * Math.pow((T),2) + 6  * T);
        double weightD = (3 *  Math.pow((T),2));

        return A.times(weightA).plus(B.times(weightB)).plus(C.times(weightC)).plus(D.times(weightD));
    }

    /**
     * Returns the normalized tangent vector to the curve at specified point
     * @param T value at which to calculate tangent vector
     * @return tangent to the curve at specified point
     */
    public Vector2d getNormalizedTangent(double T) {
        Vector2d firstDerivative = firstDerivative(T);
        return firstDerivative.div(Maths.magnitudeOf(firstDerivative));
    }

    /**
     * Returns the normalized normal vector to the curve at specified point
     * @param T value at which to calculate tangent vector
     * @return normal to the curve at specified point
     */
    public Vector2d getNormalizedNormal(double T) {
        return Maths.rotateCartesianVectorBy(getNormalizedTangent(T), Math.PI / 2);
    }

    /**
     * Uses a pre generated lookup table to estimate what the T value is for a specific arc length.
     * @param distance arc length along the curve at which we wish to find the T value
     * @return the estimated T value at which this arc length is located
     */
    public double distanceToT(double distance) {
        if (distance <= 0) return 0;
        if (distance >= totalArcLength) return totalArcLength;
        int index = 0;
        for (int i = 0; i < accuracy; i++) {
            if (lookup[i].getY() <= distance && lookup[i + 1].getY() >= distance) {
                index = i;
                break;
            }
        }
        return Maths.interpolateBetweenVectors(lookup[index], lookup[index + 1], distance).getX();
    }
}
