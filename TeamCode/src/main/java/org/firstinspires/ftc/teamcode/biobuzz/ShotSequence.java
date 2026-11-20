package org.firstinspires.ftc.teamcode.biobuzz;

final class ShotSequence {
    private boolean released;
    private boolean interrupted;

    boolean awaitingApproval() { return !released && !interrupted; }
    boolean interrupted() { return interrupted; }

    boolean update(boolean triggerPressed, boolean blocked, boolean turretStill, boolean speedReady) {
        if (!triggerPressed) {
            released = interrupted = false;
            return false;
        }
        if (blocked) {
            interrupted |= released;
            return false;
        }
        if (interrupted) return false;
        released |= turretStill && speedReady;
        return released;
    }
}
