package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.TeleOp.States.PrototypeRobotDrivetrainState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Prototype Robot", group = "PROTOTYPE")
public class PrototypeRobotEngine extends CyberarmEngine {
    private PrototypeRobot robot;
    @Override
    public void setup() {
        this.robot = new PrototypeRobot("Hello World");
        this.robot.setup();

        addState(new PrototypeRobotDrivetrainState(robot));
    }
}
