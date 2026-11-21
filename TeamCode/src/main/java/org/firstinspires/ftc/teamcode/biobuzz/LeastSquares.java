package org.firstinspires.ftc.teamcode.biobuzz;

/** Small weighted, ridge-regularized least-squares fit. */
public final class LeastSquares {
    private final double[][] rows;

    public LeastSquares(double[] penalty, double[] prior) {
        rows = new double[penalty.length][penalty.length + 1];
        for (int i = 0; i < rows.length; i++) {
            rows[i][i] = penalty[i];
            if (prior != null) rows[i][rows.length] = penalty[i] * prior[i];
        }
    }

    public void add(double[] features, double target, double weight) {
        for (int i = 0; i < rows.length; i++) {
            double factor = weight * features[i];
            for (int j = 0; j < rows.length; j++) rows[i][j] += factor * features[j];
            rows[i][rows.length] += factor * target;
        }
    }

    public double[] solve() {
        int size = rows.length;
        for (int column = 0; column < size; column++) {
            int pivot = column;
            for (int row = column + 1; row < size; row++) {
                if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
            }
            if (Math.abs(rows[pivot][column]) < 1e-12) return null;
            double[] swapped = rows[column];
            rows[column] = rows[pivot];
            rows[pivot] = swapped;
            double divisor = rows[column][column];
            for (int j = column; j <= size; j++) rows[column][j] /= divisor;
            for (int row = 0; row < size; row++) {
                if (row == column) continue;
                double factor = rows[row][column];
                for (int j = column; j <= size; j++) rows[row][j] -= factor * rows[column][j];
            }
        }
        double[] result = new double[size];
        for (int i = 0; i < size; i++) result[i] = rows[i][size];
        return result;
    }
}
