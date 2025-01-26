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
            -60,60,-27,60,-33,60,-18.8,60,2.8,60,17,60,38,59.7,55,59
    });
    public static final CubicPath BluePark = reversePath(BlueSideToScore);

    public static final CubicPath RedSideToScore = toRed(BlueSideToScore);
    public static final CubicPath RedPark = reversePath(BlueSideToScore);



    public static final CubicPath BlueBasketToLeftYellow = new CubicPath(new double[] {
            56.5,60,56.5,46.6,57,42.6,57,39.2,57,35.2,57,32.3,57,29.2,57,25
    });
    public static final CubicPath RedBasketToLeftYellow = toRed(BlueBasketToLeftYellow);



    public static final CubicPath BlueBasketToMidYellow = new CubicPath(new double[] {
            57,59.5,57,54.4,59,51.7,59,44.7,59,39,59,38,59,36,59,34 // 33 or 34
    });
    public static final CubicPath RedBasketToMidYellow = toRed(BlueBasketToMidYellow);

    public static final CubicPath MidYellowToBlueBasket = reversePath(BlueBasketToMidYellow);
    public static final CubicPath MidYellowToRedBasket = toRed(MidYellowToBlueBasket);



    public static final CubicPath BlueBasketToRightYellow = new CubicPath(new double[] {
            57.5,57,56,56.5,52,53.7,51.3,51,50.2,45.4,49.7,42.2,49,38,49,34 //33 or 34?
    });
    public static final CubicPath RedBasketToRightYellow = toRed(BlueBasketToRightYellow);

    public static final CubicPath RightYellowToBlueBasket = reversePath(BlueBasketToRightYellow);
    public static final CubicPath RightYellowToRedBasket = toRed(RightYellowToBlueBasket);
}
