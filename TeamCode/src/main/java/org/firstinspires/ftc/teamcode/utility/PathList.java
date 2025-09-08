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



    public static final CubicPath SideToScore = new CubicPath(new double[] {
            -60,60,-27,60,-33,60,-18.8,60,2.8,60,17,60,37.7,55.7,55,55
    });
    public static final CubicPath Park = reversePath(SideToScore);



    public static final CubicPath BasketToSub = new CubicPath(new double [] {
            55.25,55.25,46,50.3,43.5,48.4,42,40,40.4,31.3,38.7,25.2,33,14.3,24,12
    });
    public static final CubicPath SubToBasket = reversePath(BasketToSub);



    public static final CubicPath BasketToLeftYellow = new CubicPath(new double[] {
            55,55,56,52.4,57,50.2,58,47.6,58.7,46,59.3,44,59.7,42.7,60.25,41.25
    });



    public static final CubicPath BasketToMidYellow = new CubicPath(new double[] {
            55,55,55.6,52.7,56.6,50.6,57.2,48.6,57.5,47,58,44.6,58.7,42.6,59,40
    });
    public static final CubicPath MidYellowToBasket = reversePath(BasketToMidYellow);



    public static final CubicPath BasketToRightYellow = new CubicPath(new double[] {
            55,55,53.5,53.8,52.3,52.4,51,50.5,50.4,48.8,49.4,46.7,48.6,44,48,40
    });
    public static final CubicPath YellowToBlueBasket = reversePath(BasketToRightYellow);
}
