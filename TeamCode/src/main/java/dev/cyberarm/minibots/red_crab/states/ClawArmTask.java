package dev.cyberarm.minibots.red_crab.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class ClawArmTask extends CyberarmState {
    private final RedCrabMinibot robot;
    public ClawArmTask(RedCrabMinibot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.controlClawArm();
    }
}
