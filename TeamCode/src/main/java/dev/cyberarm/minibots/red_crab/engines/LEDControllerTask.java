package dev.cyberarm.minibots.red_crab.engines;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class LEDControllerTask extends CyberarmState {
    private final RedCrabMinibot robot;
    public LEDControllerTask(RedCrabMinibot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.ledController();
    }
}
