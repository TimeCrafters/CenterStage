package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.MinibotTeleOPBot;
import org.timecrafters.CenterStage.TeleOp.States.YellowMinibotTeleOP;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "A Yellow Minibot")

    public class MiniBotTeleOPEngine extends CyberarmEngine {
    private MinibotTeleOPBot robot;
        @Override
    public void setup() {
        this.robot = new MinibotTeleOPBot();
        this.robot.setup();

        addState(new YellowMinibotTeleOP(robot));
    }
}
