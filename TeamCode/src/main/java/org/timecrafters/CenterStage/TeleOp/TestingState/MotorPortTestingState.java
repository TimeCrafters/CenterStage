package org.timecrafters.CenterStage.TeleOp.TestingState;

import org.timecrafters.CenterStage.Common.MotorPortTestRobot;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class MotorPortTestingState extends CyberarmState {

    MotorPortTestRobot robot;
    private long lastMeasuredTime;
    private int CurrentMotorPort;
    public MotorPortTestingState(MotorPortTestRobot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        lastMeasuredTime = System.currentTimeMillis();
    }

    @Override
    public void exec() {

    }
}
