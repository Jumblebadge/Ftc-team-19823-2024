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
            -55.4,40,-31,40,-31.3,40,-22.2,40,-8,40,11.3,40,38.6,33.7,60,60
    });
    public static final CubicPath RedSideToScore = toRed(BlueSideToScore);



    public static final CubicPath BlueBasketToLeftYellow = new CubicPath(new double[] {
            60,60,61.5,56.7,64,53.3,67.5,48.3,68,43.6,69.7,38.8,69.5,35.6,70,30
    });
    public static final CubicPath RedBasketToLeftYellow = toRed(BlueBasketToLeftYellow);

    public static final CubicPath LeftYellowToBlueBasket = reversePath(BlueBasketToLeftYellow);
    public static final CubicPath LeftYellowToRedBasket = reversePath(LeftYellowToBlueBasket);



    public static final CubicPath BlueBasketToMidYellow = new CubicPath(new double[] {
            60,60,60,56.4,60,52.7,60,48.3,60,44.5,60,39.4,60,35,60,30
    });
    public static final CubicPath RedBasketToMidYellow = toRed(BlueBasketToMidYellow);

    public static final CubicPath MidYellowToBlueBasket = reversePath(BlueBasketToMidYellow);
    public static final CubicPath MidYellowToRedBasket = toRed(MidYellowToBlueBasket);



    public static final CubicPath BlueBasketToRightYellow = new CubicPath(new double[] {
            60,60,58.8,56.7,57,54,55,50.8,52.7,46.3,51.5,42.2,49.5,37,49,30
    });
    public static final CubicPath RedBasketToRightYellow = toRed(BlueBasketToRightYellow);

    public static final CubicPath RightYellowToBlueBasket = reversePath(BlueBasketToRightYellow);
    public static final CubicPath RightYellowToRedBasket = toRed(RightYellowToBlueBasket);
}
