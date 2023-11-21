package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoArmState extends CyberarmState {

    private SodiPizzaMinibotObject robot;
    private String groupName, actionName;

    public void SodiPizzaAutoDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }
    @Override
    public void exec() {
    }
}
