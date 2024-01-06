package dev.cyberarm.minibots.red_crab.engines;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public abstract class RedCrabAutonomousEngine extends CyberarmEngine {
    protected RedCrabMinibot robot;

    @Override
    public void loop() {
        Utilities.hubsClearBulkReadCache(hardwareMap);

        if (robot != null)
            robot.standardTelemetry();

        super.loop();
    }
}
