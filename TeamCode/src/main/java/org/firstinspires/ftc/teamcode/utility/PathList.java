package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.maths.CubicPath;

import java.util.Arrays;
import java.util.Collections;

public class PathList {

    public static CubicPath reversePath(CubicPath path) {
        Vector2d[] pointList = path.getControlPointList();
        Collections.reverse(Arrays.asList(pointList));
        return new CubicPath(pointList);
    }

    public static CubicPath toRed(CubicPath path) {
        Vector2d[] pointList = path.getControlPointList();
        int i = 0;
        for (Vector2d vec : pointList) {
            pointList[i] = new Vector2d(-vec.getX(), -vec.getY());
            i++;
        }
        return new CubicPath(pointList);
    }


    public static final CubicPath BlueSideToScore = new CubicPath(new double[] {
            -60,60,-27,60,-33,60,-18.8,60,2.8,60,17,60,38,57.5,53.5,57.5
    });
    public static final CubicPath RedSideToScore = toRed(BlueSideToScore);



    public static final CubicPath BlueBasketToLeftYellow = new CubicPath(new double[] {
            53.5,57.5,53.7,45.4,58.4,42.6,58.7,39.2,58.5,35.2,58.5,32.3,58.5,29.2,58.5,27
    });
    public static final CubicPath RedBasketToLeftYellow = toRed(BlueBasketToLeftYellow);



    public static final CubicPath BlueBasketToMidYellow = new CubicPath(new double[] {
            53.5,57.5,53.4,55,53.2,52.7,53.2,49,59,39,59,35.4,59,33.5,59,33
    });
    public static final CubicPath RedBasketToMidYellow = toRed(BlueBasketToMidYellow);

    public static final CubicPath MidYellowToBlueBasket = reversePath(BlueBasketToMidYellow);
    public static final CubicPath MidYellowToRedBasket = toRed(MidYellowToBlueBasket);



    public static final CubicPath BlueBasketToRightYellow = new CubicPath(new double[] {
            53.5,57.5,52.7,56.5,52,55.5,51.3,51,50.5,46.6,49.7,42.2,49,38,49,33
    });
    public static final CubicPath RedBasketToRightYellow = toRed(BlueBasketToRightYellow);

    public static final CubicPath RightYellowToBlueBasket = reversePath(BlueBasketToRightYellow);
    public static final CubicPath RightYellowToRedBasket = toRed(RightYellowToBlueBasket);
}
