package org.firstinspires.ftc.teamcode.biobuzz.shooting;

import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

/** Field targets, SOTM lead prediction, trigger interlock sequence, and autonomous volley timer. */
public final class Aiming {
    private Aiming() {}

    /** Aim solution combining ShotMap query with moving-lead bearing and line-of-sight angular rate. */
    public static final class CornerAim {
        public final double bearing, rpm;
        public final int goal;
        private final double lineRate;

        private CornerAim(int goal, ShotVectorField.Vector shot, double lineRate) {
            this.goal = goal;
            bearing = shot.bearing();
            rpm = shot.rpm();
            this.lineRate = lineRate;
        }

        public static CornerAim fromPose(ShotMap map, PinpointPose pose, boolean braking) {
            if (map == null || !map.available() || !pose.headingUsable()
                    || !positionForAim(pose)) return null;
            boolean rightHalf = -pose.left >= FieldTargets.FIELD_TILES * RobotConfig.TILE_MM / 2;
            ShotMotion motion = new ShotMotion(pose);
            ShotVectorField.Vector shot = motion.predict(map, rightHalf, braking);
            if (shot == null) return null;
            return new CornerAim(rightHalf ? 4 : 3, shot, motion.lineRate(map.goalRight(rightHalf), map.goalForward()));
        }

        public static double targetRpm(CornerAim aim) {
            return aim == null ? RobotConfig.FLYWHEEL_TARGET_RPM : aim.rpm;
        }

        public static boolean positionForAim(PinpointPose pose) {
            return pose.positionValid || pose.recentlyLocated(0.50);
        }

        public static boolean noFireZone(PinpointPose pose) {
            return pose.positionValid && pose.headingValid
                    && FieldTargets.fullyInNoFireZone(pose.forward, pose.left, pose.heading);
        }

        public static void track(Turret turret, PinpointPose pose, CornerAim aim, double yawTrim, double turn, double dt) {
            if (aim == null) turret.aim(pose, 0, 0, 0, yawTrim, turn, dt);
            else turret.aim(pose, aim.goal, aim.bearing, aim.lineRate, yawTrim, turn, dt);
        }
    }

    /** Geometric fallback goals and no-fire rectangle geometry. */
    public static final class FieldTargets {
        public static final double FIELD_TILES = 6;
        private static final double GOAL_FORWARD = 2.5;
        private static final double LEFT_GOAL_RIGHT = 2;
        private static final double RIGHT_GOAL_RIGHT = 4;
        private static final double SPLIT_RIGHT = (LEFT_GOAL_RIGHT + RIGHT_GOAL_RIGHT) / 2;
        public final int goal;
        public final double bearing;
        public final double distanceMm;
        public final double rightTiles, forwardTiles;

        private FieldTargets(int goal, double bearing, double distanceMm) {
            this.goal = goal;
            this.bearing = bearing;
            this.distanceMm = distanceMm;
            rightTiles = goal == 1 ? LEFT_GOAL_RIGHT : RIGHT_GOAL_RIGHT;
            forwardTiles = GOAL_FORWARD;
        }

        public static FieldTargets fromPose(double forwardMm, double leftMm, double heading) {
            double tile = RobotConfig.TILE_MM;
            int goal = -leftMm >= SPLIT_RIGHT * tile ? 2 : 1;
            double cos = Math.cos(heading), sin = Math.sin(heading);
            double pivotForward = forwardMm + cos * RobotConfig.TURRET_FORWARD_MM + sin * RobotConfig.TURRET_RIGHT_MM;
            double pivotLeft = leftMm + sin * RobotConfig.TURRET_FORWARD_MM - cos * RobotConfig.TURRET_RIGHT_MM;
            double dx = GOAL_FORWARD * tile - pivotForward;
            double dy = -(goal == 1 ? LEFT_GOAL_RIGHT : RIGHT_GOAL_RIGHT) * tile - pivotLeft;
            return new FieldTargets(goal, Math.atan2(dy, dx), Math.hypot(dx, dy));
        }

        public static boolean fullyInNoFireZone(double forwardMm, double leftMm, double heading) {
            double cos = Math.cos(heading), sin = Math.sin(heading);
            double halfLength = RobotConfig.ROBOT_LENGTH_MM / 2;
            double halfWidth = RobotConfig.ROBOT_WIDTH_MM / 2;
            double centerForwardOffset = halfLength - RobotConfig.CENTER_FROM_REAR_EDGE_MM;
            double centerRightOffset = halfWidth - RobotConfig.CENTER_FROM_LEFT_EDGE_MM;
            double forward = forwardMm + cos * centerForwardOffset + sin * centerRightOffset;
            double right = -leftMm - sin * centerForwardOffset + cos * centerRightOffset;
            double forwardExtent = Math.abs(cos) * halfLength + Math.abs(sin) * halfWidth;
            double rightExtent = Math.abs(sin) * halfLength + Math.abs(cos) * halfWidth;
            return right - rightExtent >= LEFT_GOAL_RIGHT * RobotConfig.TILE_MM
                    && right + rightExtent <= RIGHT_GOAL_RIGHT * RobotConfig.TILE_MM
                    && forward - forwardExtent >= GOAL_FORWARD * RobotConfig.TILE_MM
                    && forward + forwardExtent <= (FIELD_TILES - GOAL_FORWARD) * RobotConfig.TILE_MM;
        }
    }

    /** Shoot-on-the-move release delay and flight-time lead extrapolation. */
    public static final class ShotMotion {
        private final double right, forward, rightRate, forwardRate;
        private final double cos, sin, rightOrbitRate, forwardOrbitRate;

        public ShotMotion(PinpointPose pose) {
            cos = Math.cos(pose.heading);
            sin = Math.sin(pose.heading);
            double orbitForward = cos * RobotConfig.TURRET_FORWARD_MM + sin * RobotConfig.TURRET_RIGHT_MM;
            double orbitRight = -sin * RobotConfig.TURRET_FORWARD_MM + cos * RobotConfig.TURRET_RIGHT_MM;
            forward = pose.forward + orbitForward;
            right = -pose.left + orbitRight;
            rightRate = pose.positionValid ? -pose.leftVelocity : 0;
            forwardRate = pose.positionValid ? pose.forwardVelocity : 0;
            rightOrbitRate = -orbitForward * pose.yawRate;
            forwardOrbitRate = orbitRight * pose.yawRate;
        }

        public ShotVectorField.Vector predict(ShotMap map, boolean rightHalf, boolean braking) {
            double tile = RobotConfig.TILE_MM;
            if (!RobotConfig.ENABLE_MOVING_LEAD) {
                return map.sample(right / tile, forward / tile, rightHalf);
            }
            double range = Math.hypot(right - tile * map.goalRight(rightHalf), forward - tile * map.goalForward());
            double flight = flightTime(range);
            double delay = RobotConfig.SHOT_RELEASE_SECONDS;
            double vf = cos * forwardRate - sin * rightRate;
            double vr = sin * forwardRate + cos * rightRate;
            double along = lead(vf, braking ? RobotConfig.BRAKE_FORWARD_MM_S2 : 0, delay, flight);
            double across = lead(vr, braking ? RobotConfig.BRAKE_SIDE_MM_S2 : 0, delay, flight);
            double leadRight = -sin * along + cos * across;
            double leadForward = cos * along + sin * across;
            double leadNorm = Math.hypot(leadRight, leadForward);
            if (leadNorm > 450) {
                double scale = 450 / leadNorm;
                leadRight *= scale;
                leadForward *= scale;
            }
            return map.sample((right + leadRight) / tile, (forward + leadForward) / tile, rightHalf);
        }

        public double lineRate(double goalRightTiles, double goalForwardTiles) {
            return ShotMap.lineRate(right, forward, rightRate + rightOrbitRate,
                    forwardRate + forwardOrbitRate, goalRightTiles * RobotConfig.TILE_MM,
                    goalForwardTiles * RobotConfig.TILE_MM);
        }

        public static double flightTime(double distanceMm) {
            double span = Math.max(1.0, RobotConfig.FLIGHT_FAR_MM - RobotConfig.FLIGHT_NEAR_MM);
            double fraction = (distanceMm - RobotConfig.FLIGHT_NEAR_MM) / span;
            return RobotConfig.FLIGHT_NEAR_SECONDS
                    + (RobotConfig.FLIGHT_FAR_SECONDS - RobotConfig.FLIGHT_NEAR_SECONDS) * Maths.clip(fraction, 0, 1);
        }

        public static double lead(double speed, double deceleration, double delay, double flight) {
            double magnitude = Math.abs(speed);
            double moving = deceleration > 0 ? Math.min(delay, magnitude / deceleration) : delay;
            double release = Math.max(0, magnitude - deceleration * moving);
            return Math.copySign(0.5 * (magnitude + release) * moving + release * flight, speed);
        }
    }

    /** Single-press latched feeding state machine with turret / RPM / intake interlocks. */
    public static final class ShotSequence {
        private static final double IDLE_RPM = 2500;
        private final Turret turret;
        private final Flywheel shooter;
        private final Intake intake;
        private final boolean calibratedMode;

        private boolean released;
        private boolean interrupted;
        private boolean feeding;
        private boolean blocked, turretReady, speedReady, intakeReady;

        public ShotSequence(Turret turret, Flywheel shooter, Intake intake, boolean calibratedMode) {
            this.turret = turret;
            this.shooter = shooter;
            this.intake = intake;
            this.calibratedMode = calibratedMode;
        }

        public boolean update(boolean triggerPressed, boolean reverseRequested, boolean noFireZone,
                              boolean trackingTurret, boolean turretZeroLock,
                              double targetRpm, boolean rpmLockEnabled) {
            boolean triggerHeld = triggerPressed && !reverseRequested;
            if (shooter != null) {
                shooter.sample();
                boolean prepare = (!rpmLockEnabled || triggerHeld) && !noFireZone && targetRpm > 0;
                shooter.requestRpm(calibratedMode && !prepare ? IDLE_RPM : targetRpm);
            }
            blocked = reverseRequested || noFireZone || targetRpm <= 0 || intake == null || shooter == null
                    || shooter.faulted() || !shooter.rpmCalibrated() || !shooter.hasBlocker();
            boolean checkShot = triggerHeld && !blocked;
            boolean turretAligned = turret == null || !turret.calibrated() || !turret.encoderReady()
                    || !turret.motorResponding()
                    || (turretZeroLock ? (turret.atForwardZero() || turret.stableForShot())
                    : (!trackingTurret || turret.aimReady() || turret.stableForShot()));
            turretReady = checkShot && (released || interrupted || turretAligned);
            speedReady = checkShot && (!rpmLockEnabled || shooter.readyForShot(feeding));
            intakeReady = intake != null && intake.readyToFeed();
            return updateTrigger(triggerPressed, turretReady && intakeReady);
        }

        public String waitingStatus() {
            if (interrupted) return "供弹已中断，松开 RT 后再按";
            if (blocked) return "检查云台、供弹机构和飞轮";
            if (!turretReady) return "等待云台对准或停稳";
            if (!speedReady) return "等待飞轮进入 −8% 至 +12% 转速区间";
            if (!intakeReady) return "等待吸取舵机抬起";
            return null;
        }

        private boolean updateTrigger(boolean triggerPressed, boolean approvalReady) {
            if (!triggerPressed) {
                releaseTrigger();
                return false;
            }
            if (blocked) {
                interrupted |= released;
                feeding = false;
                return false;
            }
            if (interrupted) return false;
            released |= approvalReady && speedReady;
            feeding = released && speedReady;
            return feeding;
        }

        public void releaseTrigger() { released = interrupted = feeding = false; }
    }

    /** Timed autonomous shooting window bounded by feed target and deadline. */
    public static final class AutoVolley {
        private final double targetFeedSeconds;
        private final double deadline;
        private boolean feeding;
        private double feedSeconds, previousUpdate;

        public AutoVolley(double start, double routeWaitSeconds) {
            previousUpdate = start;
            targetFeedSeconds = Math.max(0.20, RobotConfig.AUTO_VOLLEY_FEED_SECONDS);
            deadline = start + Math.max(routeWaitSeconds, targetFeedSeconds)
                    + Math.max(0.0, RobotConfig.AUTO_VOLLEY_MAX_EXTRA_SECONDS);
        }

        public boolean requesting(double now) {
            if (feeding) feedSeconds += now - previousUpdate;
            previousUpdate = now;
            return now < deadline && !finished();
        }

        public void recordOutput(boolean feeding) { this.feeding = feeding; }
        public boolean finished() { return feedSeconds >= targetFeedSeconds; }
        public double feedSeconds() { return feedSeconds; }
        public double targetFeedSeconds() { return targetFeedSeconds; }
    }
}
