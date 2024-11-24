package org.firstinspires.ftc.teamcode.maths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class MedianFilter {

    private ArrayList<Double> dataSet;
    private final int filterSize;
    private boolean isInitialized = false, isEven = false;

    public MedianFilter(int filterSize) {
        this.filterSize = filterSize;
        if (filterSize % 2 == 0) isEven = true;
        dataSet = new ArrayList<>(filterSize);
    }

    public double getFilteredValue(double currentValue) {
        if (!isInitialized) {
            isInitialized = true;
            for (int i = 0; i < filterSize; i++) {
                dataSet.add(currentValue);
            }
        }

        double[] sortedSet = new double[filterSize];
        for (int i = 0; i < filterSize; i++) {
            sortedSet[i] = dataSet.get(i);
        }

        dataSet.add(currentValue);
        dataSet.remove(0);

        Arrays.sort(sortedSet);

        double output;

        if (isEven) output = ((sortedSet[filterSize / 2] + sortedSet[filterSize / 2 - 1]) / 2);
        else output = sortedSet[filterSize / 2];

        return output;
    }

}
