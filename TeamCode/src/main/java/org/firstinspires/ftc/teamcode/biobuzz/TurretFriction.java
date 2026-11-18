package org.firstinspires.ftc.teamcode.biobuzz;

/** Directional friction sampled over the taught travel. The two cable limits are distinct. */
public final class TurretFriction {
    public static final int SECTORS = 12;
    private final double[] data;
    private final boolean measured;

    public TurretFriction(CalibrationStore store) {
        this(store, TurretPlant.load(store));
    }

    TurretFriction(CalibrationStore store, TurretPlant plant) {
        double[] stored = store.readExact(CalibrationStore.TURRET_FRICTION_MAP, 2 + SECTORS * 4);
        measured = valid(stored);
        data = measured ? stored : uniform(plant);
    }

    public TurretFriction(TurretPlant plant) {
        data = uniform(plant);
        measured = false;
    }

    private static boolean valid(double[] data) {
        if (data == null || data[0] <= 0 || data[1] <= 0) return false;
        for (int i = 2; i < data.length; i++) if (data[i] < 0 || data[i] > 1) return false;
        return true;
    }

    boolean measured() { return measured; }
    private static double[] uniform(TurretPlant plant) {
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
