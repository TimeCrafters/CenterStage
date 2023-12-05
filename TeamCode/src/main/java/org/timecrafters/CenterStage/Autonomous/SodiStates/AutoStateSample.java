package org.timecrafters.CenterStage.Autonomous.SodiStates;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class AutoStateSample extends CyberarmState {

    private final boolean stateDisabled;
    PrototypeRobot robot;
    public AutoStateSample(PrototypeRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.stateDisabled = !robot.configuration.action(groupName, actionName).enabled;
    }

    @Override
    public void start() {
        //add variables that need to be reinitillized
    }

    @Override
    public void exec() {
        setHasFinished(true);
    }
}
