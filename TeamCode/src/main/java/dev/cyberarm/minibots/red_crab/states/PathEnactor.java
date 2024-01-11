package dev.cyberarm.minibots.red_crab.states;

import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfigurationError;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class PathEnactor extends CyberarmState {
    private final RedCrabMinibot robot;
    private String pathGroupName;
    private int forcePath;
    public PathEnactor(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        try {
            this.forcePath = robot.config.variable(groupName, actionName, "forcePath").value();
        } catch (TimeCraftersConfigurationError e) {
            this.forcePath = -1;
        }
    }

    @Override
    public void start() {
        // FORCE PATH FOR DEBUGGING
        if (Utilities.isBetween(forcePath, 0, 2))
            engine.blackboardSet("autonomousPath", forcePath);

        String path;
        switch (engine.blackboardGetInt("autonomousPath")) {
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

        this.pathGroupName = String.format("Autonomous_SpikePath_%s", path);
    }

    @Override
    public void exec() {
        engine.setupFromConfig(
                robot.config, "dev.cyberarm.minibots.red_crab.states", robot, robot.getClass(), pathGroupName, this);

        finished();
    }
}
