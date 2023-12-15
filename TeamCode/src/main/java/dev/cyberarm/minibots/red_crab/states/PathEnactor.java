package dev.cyberarm.minibots.red_crab.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class PathEnactor extends CyberarmState {
    private final RedCrabMinibot robot;
    private String pathGroupName;
    public PathEnactor(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;
    }

    @Override
    public void start() {
        String path;
        switch (engine.blackboardGetInt("AutonomousPath")) {
            case 1:
                path = "CENTER";
                break;
            case 2:
                path = "RIGHT";
                break;
            default:
                path = "LEFT";
                break;
        }

        this.pathGroupName = String.format("AutonomousPixelPath_%s", path);
    }

    @Override
    public void exec() {
        engine.setupFromConfig(
                robot.config, "dev.cyberarm.minibots.red_crab.states", robot, robot.getClass(), pathGroupName, this);

        finished();
    }
}
