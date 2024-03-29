package dev.cyberarm.minibots.red_crab.engines;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public abstract class RedCrabEngine extends CyberarmEngine {
    protected RedCrabMinibot robot;

    @Override
    public void loop() {
        Utilities.hubsClearBulkReadCache(hardwareMap);

        super.loop();

        robot.standardTelemetry();
    }

    @Override
    public void stop() {
        robot.shutdown();

        super.stop();
    }
}
