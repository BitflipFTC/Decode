package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

/**
 * Implements a lookup table that uses linear interpolation to estimate output values
 * (angle and velocity) based on a given input distance.
 * * The implementation enforces clamping, meaning any input outside the defined distance
 * range is treated as the closest valid endpoint distance.
 */
public class InterpolatedLookupTable {

    /**
     * Helper class to store the lower and upper bounds of a segment in the lookup table.
     * Stores both the independent variable's value and its index in the array.
     */
    static class Bounds {
        /** The distance value at the lower index. */
        double lowerValue = 0;
        /** The distance value at the upper index. */
        double upperValue = 0;
        /** The lower index in the table (x1). */
        double lowerIndex = 0;
        /** The upper index in the table (x2). */
        double upperIndex = 0;

        /**
         * Constructs a Bounds object with specified values and indices.
         * @param lowerValue The value at the lower index.
         * @param upperValue The value at the upper index.
         * @param lowerIndex The array index of the lower value.
         * @param upperIndex The array index of the upper value.
         */
        protected Bounds (double lowerValue, double upperValue, double lowerIndex, double upperIndex) {
            this.lowerValue = lowerValue;
            this.upperValue = upperValue;
            this.lowerIndex = lowerIndex;
            this.upperIndex = upperIndex;
        }

        /**
         * Default constructor for Bounds, initializes all fields to 0.
         */
        protected Bounds () {}
    }

    /**
     * Helper class to encapsulate the three necessary arrays for the lookup table.
     */
    static class Dataset {
        /** The independent variable array (Distance). Must be strictly increasing. */
        double[] inputTable;
        /** The first dependent variable array (Angle). */
        double[] outputTable;


        /**
         * Constructs a Dataset object.
         * @param inputTable The array of distances.
         * @param outputTable The array of angles corresponding to the distances.
         */
        protected Dataset(double[] inputTable, double[] outputTable) {
            this.inputTable = inputTable;
            this.outputTable = outputTable;
        }
    }

    /** The encapsulated lookup table data. */
    Dataset dataset;

    /**
     * Constructs the InterpolatedLookupTable from three arrays of any length
     * @param inputTable a {@code double[]}, sorted in an ascending order
     * @param outputTable a {@code double[]}, sorted in any order
     * </ul>
     */
    public InterpolatedLookupTable(double[] inputTable, double[] outputTable) {
        this.dataset = new Dataset(inputTable, outputTable);
    }

    /**
     * Calculates the interpolated/clamped angle and velocity for a given distance input.
     * * The input is first clamped to the range of the distanceTable.
     * A segment is found where the input lies, and linear interpolation is used
     * to estimate the corresponding output values.
     * @param input The distance value to look up.
     * @return new {@code FlywheelState()} containing angle, velocity, and estimated shot time
     */
    public double calculate(double input) {
        double[] inputTable = dataset.inputTable;
        double[] outputTable = dataset.outputTable;

        double output;

        Bounds bounds = new Bounds();

        // 1. Clamping Logic: Ensure input is within the table's range.
        if (input > inputTable[inputTable.length - 1]) {
            input = inputTable[inputTable.length - 1];
        } else if (input < inputTable[0]) {
            input = inputTable[0];
        }

        // 2. Find Bounding Indices
        for (int i = 1; i < inputTable.length; i++) {
            double lastVal = inputTable[i-1];
            double currentVal = inputTable[i];

            // Finds the segment where V_lower <= input <= V_upper
            if ((currentVal >= input && lastVal < input) || lastVal == input) {
                bounds.lowerValue = lastVal;
                bounds.upperValue = currentVal;
                bounds.lowerIndex = i-1;
                bounds.upperIndex = i;

                break;
            }
        }

        // estimated index - where the passed in distance WOULD BE in the dist array
        // The fractional index is calculated based on the input's position between the bounds.
        // Formula: index = I_lower + (V_input - V_lower) / (V_upper - V_lower)
        double index = (input - bounds.lowerValue) / (bounds.upperValue - bounds.lowerValue) + bounds.lowerIndex;

        // 3. Interpolate Output Values
        output = interpolate(index, (int) bounds.lowerIndex, (int) bounds.upperIndex, outputTable);

        return output;
    }

    /**
     * Performs a single linear interpolation step.
     * Uses the point-slope form: y = m * (x - x1) + y1
     * where x is the fractional index, x1 is lowerBound, and y1 is table[lowerBound].
     * * @param index The fractional index (x) to interpolate at.
     * @param lowerBound The integer index of the lower point (x1).
     * @param upperBound The integer index of the upper point (x2).
     * @param table The array containing the dependent variable values (y).
     * @return The interpolated value (y).
     */
    private double interpolate (double index, int lowerBound, int upperBound, double[] table) {
        // point - slope linear equation
        // y2 = table[upperBound];
        // y1 = table[lowerBound];
        // x2 = upperBound;
        // x1 = lowerBound;

        // Calculate the slope (m)
        double m = (table[upperBound] - table[lowerBound]) / (upperBound - lowerBound);

        // Return the interpolated value
        return (m * (index - lowerBound) + table[lowerBound]);
    }
}