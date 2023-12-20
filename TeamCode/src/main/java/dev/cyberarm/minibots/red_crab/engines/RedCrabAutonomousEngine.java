package dev.cyberarm.minibots.red_crab.engines;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;

public abstract class RedCrabAutonomousEngine extends CyberarmEngine {
    @Override
    public void loop() {
        Utilities.hubsClearBulkReadCache(hardwareMap);

        super.loop();
    }
}
