package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.CenterStage.Autonomous.SodiStates.AutoStateScrimmage;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Scrimmage Auto", group = "PROTOTYPE", preselectTeleOp = "Prototype Robot")

public class ServoTestEngine extends CyberarmEngine {

    PrototypeRobot robot;
    @Override
    public void setup() {
        this.robot = new PrototypeRobot("Hello World");
        this.robot.setup();

        addState(new AutoStateScrimmage(robot));
    }
}
