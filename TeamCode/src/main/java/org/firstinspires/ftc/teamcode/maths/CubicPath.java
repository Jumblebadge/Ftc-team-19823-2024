package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.Vector2d;

public class CubicPath {

    private Vector2d temp = new Vector2d(0,0);
    public Bezier[] beziers = {new Bezier(temp, temp, temp, temp), new Bezier(temp, temp, temp, temp), new Bezier(temp, temp, temp, temp)};
    public double guessT = 0, arcLength = 0;
    private double totalArcLength;
    Vector2d[] controlPoints;
    double[] arcLengths = new double[beziers.length];

    public CubicPath(double[] controlPoints) {
        this.controlPoints = Maths.pointListToVectorList(controlPoints);
        beziers[0].setControlPoints(this.controlPoints[0], this.controlPoints[1], this.controlPoints[2], this.controlPoints[3]);
        beziers[1].setControlPoints(this.controlPoints[3], this.controlPoints[3].times(2).minus(this.controlPoints[2]), this.controlPoints[4], this.controlPoints[5]);
        beziers[2].setControlPoints(this.controlPoints[5], this.controlPoints[5].times(2).minus(this.controlPoints[4]), this.controlPoints[6], this.controlPoints[7]);
        calculateTotalArcLength();
    }

    public void setControlPointCoordinates(double[] controlPointCoordinates) {
        controlPoints = Maths.pointListToVectorList(controlPointCoordinates);
        beziers[0].setControlPoints(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3]);
        beziers[1].setControlPoints(controlPoints[3], controlPoints[3].times(2).minus(controlPoints[2]), controlPoints[4], controlPoints[5]);
        beziers[2].setControlPoints(controlPoints[5], controlPoints[5].times(2).minus(controlPoints[4]), controlPoints[6], controlPoints[7]);
        calculateTotalArcLength();
    }

    public Vector2d getPoint(double T) {
        if (T < 0) { T = 0; }
        if (T >= 3) { T = 2.9999; }
        return beziers[(int) T].getPoint(T - Math.floor(T));
    }

    public Vector2d getControlPoint(int index) {
        return controlPoints[index];
    }

    public Vector2d getNormalizedTangent(double T) { return beziers[(int) T].getNormalizedTangent(T - Math.floor(T)); }

    public Vector2d getNormalizedNormal(double T) { return beziers[(int) T].getNormalizedNormal(T - Math.floor(T)); }

    public double getTotalArcLength() {
        return totalArcLength;
    }

    public void calculateTotalArcLength() {
        double total = 0;
        int i = 0;
        for (Bezier bezier : beziers) {
            double length = bezier.getTotalArcLength();
            total += length;
            arcLengths[i] = length;
            i++;
        }
        totalArcLength = total;
    }

    public int whichBezierFromDistance(double distance) {
        if (distance <= arcLengths[0]) {
            return 0;
        }
        else if (/*arcLengths[0] <= distance &&*/ distance <= arcLengths[0] + arcLengths[1]) {
            return 1;
        }
        else if (arcLengths[0] + arcLengths[1] <= distance && distance <= arcLengths[0] + arcLengths[1] + arcLengths[2]) {
            return 2;
        }
        //return -(int) distance
        return 2;
    }

    public double distanceToT(double distance) {
        int bezier = whichBezierFromDistance(distance);
        double minus = 0;
        if (bezier == 1) { minus = arcLengths[0]; }
        if (bezier == 2) { minus = arcLengths[0] + arcLengths[1]; }
        return (beziers[bezier].distanceToT(distance - minus)) + bezier;
    }

    public Vector2d findClosestPointOnPath(Vector2d Robot) {
        for (int i = 0; i <= 10; i++) {
            Vector2d guess = getPoint(guessT);
            Vector2d robotVector = new Vector2d(Robot.x - guess.x, Robot.y - guess.y);
            Vector2d normalizedTangent = getNormalizedTangent(guessT);
            double totalArcLength = getTotalArcLength();
            arcLength += normalizedTangent.dot(robotVector);
            if (arcLength < 0) { guessT = 0; arcLength = 0; }
            if (arcLength >= totalArcLength) { arcLength = totalArcLength - 0.01; }
            guessT = distanceToT(arcLength);
            if (guessT > 2.999) { guessT = 2.999; }
        }
        return getPoint(guessT);
    }

}
