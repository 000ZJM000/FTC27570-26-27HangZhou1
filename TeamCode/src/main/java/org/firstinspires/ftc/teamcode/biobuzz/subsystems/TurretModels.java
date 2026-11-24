package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;

/** Turret physical models: plant response, friction table, cable range, predictive brake, and Kalman observer. */
public final class TurretModels {
    private TurretModels() {}

    /** Per-direction duty versus speed model identified from short, open-loop pulses. */
    public static final class Plant {
        private static final double[] DEFAULT = {
                0.09, 0.068, 0.11, 0.22, 0.05, 0.09, 0.068, 0.11, 0.22, 0.05, 0.85
        };
        private final double[] values;
        private final boolean identified;

        public Plant(double... values) {
            identified = valid(values);
            this.values = (identified ? values : DEFAULT).clone();
        }

        public static Plant load(CalibrationStore store) {
            return new Plant(store.readExact(CalibrationStore.TURRET_PLANT, DEFAULT.length));
        }

        private static boolean valid(double[] values) {
            if (values == null || values.length != DEFAULT.length || !Maths.finite(values)) return false;
            for (int i : new int[]{0, 5}) {
                if (values[i] <= 0 || values[i] > 1 || values[i + 1] < 0 || values[i + 1] > 1
                        || values[i + 2] <= 0 || values[i + 3] <= 0 || values[i + 4] < 0) return false;
            }
            return values[10] > 0;
        }

        public boolean identified() { return identified; }
        private int side(int direction) { return direction > 0 ? 0 : 5; }
        public double breakaway(int direction) { return values[side(direction)]; }
        public double rolling(int direction) { return values[side(direction) + 1]; }
        public double kV(int direction) { return values[side(direction) + 2]; }
        public double timeConstant(int direction) { return values[side(direction) + 3]; }
        public double delay(int direction) { return values[side(direction) + 4]; }
        public double baseline() { return values[10]; }
        public double[] values() { return values.clone(); }

        public double[] pidSeed(double bandwidthScale) {
            double tau = Math.max(timeConstant(1), timeConstant(-1));
            double kV = (kV(1) + kV(-1)) * 0.5;
            double delay = Math.max(delay(1), delay(-1));
            double slowPole = Math.min(4.8, Math.min(0.56 / (0.095 + delay), 0.88 / tau + 0.55));
            double rate = slowPole * Maths.clip(bandwidthScale, 0.48, 1.25);
            double fastPole = Math.max(2.15 * rate, 1 / tau - rate);
            double p = tau * kV * rate * fastPole;
            double d = Math.max(0, tau * kV * (rate + fastPole) - kV);
            return new double[]{p, Maths.clip(0.13 * p, 0.012, 0.22), d};
        }
    }

    /** Directional friction sampled over the taught travel. */
    public static final class Friction {
        public static final int SECTORS = 12;
        private final double[] data;
        private final boolean measured;

        public Friction(CalibrationStore store) {
            this(store, Plant.load(store));
        }

        public Friction(CalibrationStore store, Plant plant) {
            double[] stored = store.readExact(CalibrationStore.TURRET_FRICTION_MAP, 2 + SECTORS * 4);
            measured = valid(stored);
            data = measured ? stored : uniform(plant);
        }

        public Friction(Plant plant) {
            data = uniform(plant);
            measured = false;
        }

        private static boolean valid(double[] data) {
            if (data == null || data[0] <= 0 || data[1] <= 0) return false;
            for (int i = 2; i < data.length; i++) if (data[i] < 0 || data[i] > 1) return false;
            return true;
        }

        public boolean measured() { return measured; }

        private static double[] uniform(Plant plant) {
            double[] fallback = new double[2 + SECTORS * 4];
            for (int direction : new int[]{1, -1}) {
                int side = side(direction);
                fallback[side] = plant.kV(direction);
                for (int sector = 0; sector < SECTORS; sector++) {
                    int i = index(sector, direction);
                    fallback[i] = plant.breakaway(direction);
                    fallback[i + 1] = plant.rolling(direction);
                }
            }
            return fallback;
        }

        public static int side(int direction) { return direction > 0 ? 0 : 1; }
        public static int index(int sector, int direction) { return 2 + 4 * sector + 2 * side(direction); }
        public double[] values() { return data.clone(); }
        public double velocityGain(int direction) { return data[side(direction)]; }

        public static double width(double span) {
            return (span - 2 * RobotConfig.TURRET_MARGIN) / SECTORS;
        }

        public static double center(int sector, double span) {
            return RobotConfig.TURRET_MARGIN + (sector + 0.5) * width(span);
        }

        public double duty(double angle, double span, int direction, double speed) {
            double bin = Maths.clip((angle - RobotConfig.TURRET_MARGIN) / width(span) - 0.5, 0, SECTORS - 1);
            int a = (int) bin, b = Math.min(a + 1, SECTORS - 1);
            int i = index(a, direction), j = index(b, direction);
            double fraction = bin - a;
            double moving = Maths.clip(Math.abs(speed) / 0.55, 0, 1);
            double start = data[i] + fraction * (data[j] - data[i]);
            double running = data[i + 1] + fraction * (data[j + 1] - data[i + 1]);
            return start + moving * (running - start);
        }
    }

    /** Soft travel limits and predictive wrap-around rewind planner. */
    public static final class Range {
        private final double low, high;
        private int direction;
        private boolean rewindTargetMoves;
        private double previousTarget = Double.NaN;
        public boolean rewinding, reachable, predicted;

        public Range(double span) {
            low = RobotConfig.TURRET_MARGIN;
            high = Math.min(span, Maths.TURN) - RobotConfig.TURRET_MARGIN;
        }

        public void reset() {
            finishRewind();
            reachable = false;
            previousTarget = Double.NaN;
        }

        public void finishRewind() {
            direction = 0;
            rewinding = predicted = rewindTargetMoves = false;
        }

        public boolean rewindTargetMoves() { return rewindTargetMoves; }

        public double target(double bearing, double current, double relativeRate, double lookahead) {
            double equivalent = equivalent(bearing);
            reachable = equivalent <= high;
            if (predicted) {
                double source = direction > 0 ? low : high;
                if (reachable && (equivalent - source) * direction > Math.PI) {
                    predicted = false;
                } else {
                    double acrossGap = -direction * Maths.wrap(bearing - source);
                    double rateAcrossGap = -direction * relativeRate;
                    double gap = Maths.TURN - (high - low);
                    if (rateAcrossGap <= 0.20 || acrossGap + rateAcrossGap * lookahead < gap - Math.toRadians(4)) {
                        finishRewind();
                        previousTarget = reachable ? equivalent : source;
                        return previousTarget;
                    }
                }
            }
            if (!rewinding) {
                double forecast = equivalent(bearing + relativeRate * lookahead);
                double nearCurrent = current + Maths.wrap(bearing - current);
                double edge = relativeRate > 0 ? high : low;
                boolean atEdge = Math.abs(current - edge) <= Math.toRadians(26)
                        && Math.abs(previousTarget - edge) <= Math.toRadians(26);
                boolean towardEdge = (nearCurrent - edge) * Math.signum(relativeRate) >= -Math.toRadians(12);
                boolean crosses = Math.abs(relativeRate) > 0.20 && atEdge && towardEdge && forecast <= high
                        && (edge - forecast) * Math.signum(relativeRate) > Math.PI;
                if (crosses) start(-Math.signum(relativeRate), true);
                else if (reachable && Math.abs(equivalent - previousTarget) > Math.PI
                        && Math.abs(equivalent - current) > Math.PI) {
                    start(Math.signum(equivalent - current), false);
                }
                if (!rewinding) {
                    double reference = Double.isFinite(previousTarget) ? previousTarget : current;
                    double nearReference = reference + Maths.wrap(bearing - reference);
                    previousTarget = reachable ? equivalent : Maths.clip(nearReference, low, high);
                    return previousTarget;
                }
            }
            double destination = direction > 0 ? high : low;
            double branch = destination + Maths.wrap(bearing - destination);
            if (reachable && (equivalent - destination) * direction > -Math.PI) branch = equivalent;
            rewindTargetMoves = branch > low && branch < high;
            previousTarget = Maths.clip(branch, low, high);
            return previousTarget;
        }

        private void start(double direction, boolean predicted) {
            this.direction = (int) direction;
            this.predicted = predicted;
            rewinding = true;
        }

        private double equivalent(double bearing) {
            double offset = (bearing - low) % Maths.TURN;
            return low + (offset < 0 ? offset + Maths.TURN : offset);
        }
    }

    /** Predictive braking feedforward based on identified plant delay and deceleration. */
    public static final class Brake {
        private static final double MAX_DUTY = 0.38;

        private Brake() {}

        public static double duty(double error, double relativeRate, double frictionDuty, double appliedDuty,
                                  double kV, double tau, double delay) {
            if (Math.abs(relativeRate) < 0.20 || (error * relativeRate < 0
                    && Math.abs(error) > Math.toRadians(2))) return 0;
            double speed = Math.abs(relativeRate);
            double reaction = Maths.clip(delay + 0.025 + Math.max(0, Math.copySign(1, relativeRate)
                    * appliedDuty) / 8, 0.025, 0.23);
            double deceleration = (0.16 + 0.65 * kV * speed) / Math.max(0.004, kV * tau);
            double stopping = reaction * speed + speed * speed / (2 * deceleration);
            double blend = Maths.clip((stopping - Math.abs(error)) / Math.max(0.02, 0.72 * stopping), 0, 1);
            double weight = blend * blend * (3 - 2 * blend) * Maths.clip((speed - 0.20) / 0.36, 0, 1);
            double brake = Math.min(MAX_DUTY, 0.055 + 0.65 * Math.abs(frictionDuty) + 0.78 * kV * speed);
            return -Math.copySign(weight * brake, relativeRate);
        }
    }

    /** Kalman observer for turret angle, angular velocity, and disturbance bias. */
    public static final class Observer {
        private static final double MEASUREMENT_VARIANCE = 0.0004 * 0.0004;
        private final double[][] covariance = new double[3][3];
        private final double[][] transition = new double[3][3];
        private final double[][] work = new double[3][3];
        private final double[][] predicted = new double[3][3];
        public double angle, velocity, bias;

        public void reset(double angle, double velocity) {
            this.angle = angle;
            this.velocity = velocity;
            bias = 0;
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) covariance[i][j] = 0;
            covariance[0][0] = MEASUREMENT_VARIANCE;
            covariance[1][1] = 0.12;
            covariance[2][2] = 0.0004;
        }

        public void update(double measured, double measuredRate, double dt, double applied,
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
}
