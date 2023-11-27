package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoSecDriveState extends CyberarmState {

    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;

    public SodiPizzaAutoSecDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    @Override
    public void start() {

    }

    @Override
    public void exec() {

    }
}
