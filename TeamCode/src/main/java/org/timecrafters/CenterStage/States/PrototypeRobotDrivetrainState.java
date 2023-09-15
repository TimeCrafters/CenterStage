package org.timecrafters.CenterStage.States;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class PrototypeRobotDrivetrainState extends CyberarmState {
    private PrototypeRobot robot;
    public PrototypeRobotDrivetrainState(PrototypeRobot robot) {
        this.robot = robot;
    }
    @Override
    public void exec() {
    }
}
