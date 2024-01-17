package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;

import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

@Config
public class ClawArmControlTask extends CyberarmState {

    CompetitionRobotV1 robot;

    public ClawArmControlTask(CompetitionRobotV1 robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {robot.clawArmControl();}
}
