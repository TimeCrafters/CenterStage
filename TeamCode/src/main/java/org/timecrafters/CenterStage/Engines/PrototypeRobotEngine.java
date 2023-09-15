package org.timecrafters.CenterStage.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.States.PrototypeRobotDrivetrainState;
import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Prototype Robot", group = "PROTOTYPE")
public class PrototypeRobotEngine extends CyberarmEngine {
    private PrototypeRobot robot;
    @Override
    public void setup() {
        this.robot = new PrototypeRobot("Hello World");

        addState(new PrototypeRobotDrivetrainState(robot));
    }
}