package org.firstinspires.ftc.teamcode.biobuzz;

final class ShotSequence {
    private boolean released;
    private boolean interrupted;
    private boolean feeding;

    boolean awaitingApproval() { return !released && !interrupted; }
    boolean interrupted() { return interrupted; }
    boolean feeding() { return feeding; }

    boolean update(boolean triggerPressed, boolean blocked, boolean turretStill, boolean speedReady) {
        if (!triggerPressed) {
            released = interrupted = feeding = false;
            return false;
        }
        if (blocked) {
            interrupted |= released;
            feeding = false;
            return false;
        }
        if (interrupted) return false;
        released |= turretStill && speedReady;
        feeding = released && speedReady;
        return feeding;
    }
}
