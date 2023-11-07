package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.MiniBTeleOPBot;
import org.timecrafters.CenterStage.TeleOp.States.BlackMiniTeleOP;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Black Minibot")

public class MiniBTeleOPEngine extends CyberarmEngine {
    private MiniBTeleOPBot robot;

    @Override
    public void setup() {
        this.robot = new MiniBTeleOPBot();
        this.robot.setup();

        addState(new BlackMiniTeleOP(robot));
    }
}
