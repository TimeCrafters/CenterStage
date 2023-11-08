package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.MiniYTeleOPBot;
import org.timecrafters.CenterStage.TeleOp.States.MiniYellowTeleOPv2;
import org.timecrafters.CenterStage.TeleOp.States.YellowMinibotTeleOP;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "A Yellow Minibot")

    public class MiniYTeleOPEngine extends CyberarmEngine {
    private MiniYTeleOPBot robot;

    @Override
    public void setup() {
        this.robot = new MiniYTeleOPBot();
        this.robot.setup();

        addState(new MiniYellowTeleOPv2(robot));
    }
}
