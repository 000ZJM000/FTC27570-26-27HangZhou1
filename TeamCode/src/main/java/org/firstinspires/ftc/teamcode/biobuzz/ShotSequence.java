package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.concurrent.TimeUnit;

final class ShotSequence {
    private boolean channelRunning;
    private long channelStarted;

    void updateChannel(boolean running, long now) {
        if (running && !channelRunning) channelStarted = now;
        channelRunning = running;
    }

    boolean canOpenBlocker(boolean speedReady, boolean aimReady, long now) {
        return channelRunning && speedReady && aimReady
                && now - channelStarted >= TimeUnit.MILLISECONDS.toNanos(RobotConfig.SHOT_FEED_DELAY_MS);
    }
}
