package dev.cyberarm.minibots.red_crab.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class StrafeMove extends CyberarmState {
    private final RedCrabMinibot robot;
    private final String groupName, actionName;
    public StrafeMove(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void exec() {

    }
}
