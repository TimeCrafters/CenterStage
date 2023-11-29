package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.Common.XDrivetrainBot;
import org.timecrafters.CenterStage.TeleOp.States.PrototypeRobotDrivetrainState;
import org.timecrafters.CenterStage.TeleOp.States.XDrivetrainRobotState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "PID Heading Lock Robot", group = "PROTOTYPE")
public class XDriveTrainRobotEngine extends CyberarmEngine {
    private XDrivetrainBot robot;
    @Override
    public void setup() {
        this.robot = new XDrivetrainBot("Hello World");
        this.robot.setup();

        addState(new XDrivetrainRobotState(robot));
    }
}
