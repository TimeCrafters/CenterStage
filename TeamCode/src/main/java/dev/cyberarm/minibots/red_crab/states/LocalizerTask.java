package dev.cyberarm.minibots.red_crab.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class LocalizerTask extends CyberarmState {
    private final RedCrabMinibot robot;

    public LocalizerTask(RedCrabMinibot robot) {
        this.robot = robot;
    }
    @Override
    public void exec() {
        Utilities.hubsClearBulkReadCache(engine.hardwareMap);

        if (RedCrabMinibot.localizer != null) {
            RedCrabMinibot.localizer.integrate();
        }
    }
}
