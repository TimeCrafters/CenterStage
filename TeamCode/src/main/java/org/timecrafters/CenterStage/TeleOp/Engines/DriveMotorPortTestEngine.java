package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.MotorPortTestRobot;
import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.TeleOp.TestingState.MotorPortTestingState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp (name = "Motor Port Test")
public class DriveMotorPortTestEngine extends CyberarmEngine {

    MotorPortTestRobot robot;
    @Override
    public void setup() {
        robot = new MotorPortTestRobot("Hello World");

    }
}
