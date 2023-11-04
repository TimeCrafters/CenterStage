package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.MiniYTeleOPBot;
import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.TeleOp.States.ArmPosTest;
import org.timecrafters.CenterStage.TeleOp.States.YellowMinibotTeleOP;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "arm test prototype bot")

    public class ArmPosTestEngine extends CyberarmEngine {
    private PrototypeRobot robot;
        @Override
    public void setup() {
        this.robot = new PrototypeRobot("hello world");
        this.robot.setup();

        addState(new ArmPosTest(robot));
    }
}
