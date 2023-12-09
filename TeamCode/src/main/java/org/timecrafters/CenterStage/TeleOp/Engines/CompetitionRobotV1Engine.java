package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.TeleOp.States.CompetitionTeleOpState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Competition V1 TeleOp", group = "Competition V1")
public class CompetitionRobotV1Engine extends CyberarmEngine {
    private CompetitionRobotV1 robot;
    @Override
    public void setup() {
        this.robot = new CompetitionRobotV1("Hello World");
        this.robot.setup();

        addState(new CompetitionTeleOpState(robot));
    }
}
