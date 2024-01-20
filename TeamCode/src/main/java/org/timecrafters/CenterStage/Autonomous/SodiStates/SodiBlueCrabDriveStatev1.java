package org.timecrafters.CenterStage.Autonomous.SodiStates;

import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiBlueCrabDriveStatev1 extends CyberarmState {
    final private CompetitionRobotV1 robot;

    public SodiBlueCrabDriveStatev1() {
        robot = new CompetitionRobotV1("Assignment");
        robot.setup();
    }
    @Override
    public void init() {

        

    }

    @Override
    public void exec() {

    }
}
