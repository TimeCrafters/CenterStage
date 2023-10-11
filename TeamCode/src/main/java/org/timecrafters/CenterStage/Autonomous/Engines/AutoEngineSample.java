package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Disabled
@Autonomous(name = "Sample", group = "Sample", preselectTeleOp = "TeleOp")

public class AutoEngineSample extends CyberarmEngine {

    PrototypeRobot robot;
    @Override
    public void setup() {
        robot = new PrototypeRobot("Hello World");

        setupFromConfig(
                robot.configuration,
                "org.timecrafters.Autonomous.States",
                robot,
                PrototypeRobot.class,
                "Sample Auto");
    }
}
