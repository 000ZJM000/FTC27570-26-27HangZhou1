package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Kalman observer for angle, angular velocity and a slowly changing equivalent motor duty.
 * The only measurement is the encoder angle. The input is the duty that was actually sent.
 */
final class TurretObserver {
    // Trust the through-bore angle: this allowance is about two encoder counts at the turret.
    private static final double MEASUREMENT_VARIANCE = 0.0004 * 0.0004;
    private final double[][] covariance = new double[3][3];
    private final double[][] transition = new double[3][3];
    private final double[][] work = new double[3][3];
    private final double[][] predicted = new double[3][3];
    double angle, velocity, bias;

    void reset(double angle, double velocity) {
        this.angle = angle;
        this.velocity = velocity;
        bias = 0;
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) covariance[i][j] = 0;
        covariance[0][0] = MEASUREMENT_VARIANCE;
        covariance[1][1] = 0.12;
        covariance[2][2] = 0.0004;
    }

    void update(double measured, double measuredRate, double dt, double applied,
                double friction, double kV, double tau) {
        if (dt > 0.10) {
            reset(measured, measuredRate);
            return;
        }
        double drive = applied - friction;
        boolean learn = Math.abs(measuredRate) > 0.24 && Math.abs(applied) < 0.88;
        double decay = Math.exp(-dt / tau);
        double travel = tau * (1 - decay);
        double angleGain = (dt - travel) / kV;
        double rateGain = (1 - decay) / kV;
        double biasDecay = Math.exp(-dt / (learn ? 1.7 : 0.22));
        angle += travel * velocity + angleGain * (drive + bias);
        velocity = decay * velocity + rateGain * (drive + bias);
        bias *= biasDecay;
        transition[0][0] = 1;
        transition[0][1] = travel;
        transition[1][1] = decay;
        transition[0][2] = angleGain;
        transition[1][2] = rateGain;
        transition[2][2] = biasDecay;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double value = 0;
                for (int k = 0; k < 3; k++) value += transition[i][k] * covariance[k][j];
                work[i][j] = value;
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double value = 0;
                for (int k = 0; k < 3; k++) value += work[i][k] * transition[j][k];
                predicted[i][j] = value;
            }
        }
        predicted[0][0] += 0.000025 * dt;
        predicted[1][1] += 0.45 * dt;
        predicted[2][2] += 0.005 * dt;
        double variance = predicted[0][0] + MEASUREMENT_VARIANCE;
        double innovation = Maths.clip(measured - angle, -0.15, 0.15);
        angle += predicted[0][0] / variance * innovation;
        velocity += predicted[1][0] / variance * innovation;
        if (learn) bias = Maths.clip(bias + predicted[2][0] / variance * innovation, -0.15, 0.15);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                covariance[i][j] = predicted[i][j] - predicted[i][0] * predicted[0][j] / variance;
            }
        }
    }
}
