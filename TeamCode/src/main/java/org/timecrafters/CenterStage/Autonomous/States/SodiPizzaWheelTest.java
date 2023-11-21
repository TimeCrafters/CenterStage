package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaWheelTest extends CyberarmState {
    private SodiPizzaMinibotObject robot;
    private String groupName, actionName;

    public SodiPizzaWheelTest() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    @Override
    public void exec() {

        
    }
}
