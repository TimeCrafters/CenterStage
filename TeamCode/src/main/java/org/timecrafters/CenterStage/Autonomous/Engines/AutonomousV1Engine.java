package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "AutonomousV1", group = "Autonomous" /*, preselectTeleOp = "TeleOp"*/)

public class AutonomousV1Engine extends CyberarmEngine {

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

        // Camera Looks and recognizes the position
        // -----------------------------------------------------------------------------------------------camera model differentiation state

        // Drive forward
        // ----------------------------------------------------------------------------------------------------------------------drive state

        // rotate to the direction of the designated pixel placement location
        // -------------------------------------------------------------------------------------------------------------------rotation state

        // place the first pixel
        // -------------------------------------------------------------------------------------------------------collector open close state

        // Back Up
        // ----------------------------------------------------------------------------------------------------------------------drive state

        // rotate towards the backdrop
        // -------------------------------------------------------------------------------------------------------------------rotation state

        // drive forward to the backdrop
        // ----------------------------------------------------------------------------------------------------------------------drive state

        // strafe to the position of the pixel
        // ----------------------------------------------------------------------------------------------------------------------drive state

        // flip shoulder and elbow depositor out to deposit
        // ---------------------------------------------------------------------------------------------------shoulder, elbow movement state

        // deposit
        // -------------------------------------------------------------------------------------------------------depositor open close state

        // strafe over to the inside area of the backstage to park
        // ---------------------------------------------------------------------------------------------------------------------drive state
    }
}
