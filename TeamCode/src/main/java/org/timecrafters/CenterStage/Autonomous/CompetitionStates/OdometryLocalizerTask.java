package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;

import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class OdometryLocalizerTask extends CyberarmState {

    CompetitionRobotV1 robot;
    public OdometryLocalizerTask(CompetitionRobotV1 robot) {
        this.robot = robot;
    }
    @Override
    public void exec() {robot.OdometryLocalizer();}
}
