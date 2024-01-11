package dev.cyberarm.minibots.red_crab.states;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;
import dev.cyberarm.minibots.red_crab.SpikeMarkDetectorVisionProcessor;
import dev.cyberarm.minibots.red_crab.TeamPropVisionProcessor;

public class PathSelector extends CyberarmState {
    private final RedCrabMinibot robot;
    private final int timeoutMS;
    private final int path;
    private final double minConfidence;

    public PathSelector(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeoutMS").value();
        this.path = robot.config.variable(groupName, actionName, "path").value();
        this.minConfidence = robot.config.variable(groupName, actionName, "minConfidence").value();
    }

    @Override
    public void init() {
        robot.teamProp = new TeamPropVisionProcessor();
        robot.visionPortal = VisionPortal.easyCreateWithDefaults(
                engine.hardwareMap.get(WebcamName.class, robot.webcamName), robot.teamProp);

        engine.blackboardSet("autonomousPath", path);
    }

    @Override
    public void exec() {
        // If we've got enough Saturation for our min confidence then do the needful.
        if (robot.teamProp.getHighestSaturation() >= minConfidence) {
            switch (robot.teamProp.getLocation()) {
                case LEFT:
                    engine.blackboardSet("autonomousPath", 0);
                    break;
                case CENTER:
                    engine.blackboardSet("autonomousPath", 1);
                    break;
                case RIGHT:
                    engine.blackboardSet("autonomousPath", 2);
                    break;
            }
        } else
        {
            engine.blackboardSet("autonomousPath", path); // min confidence not meant, default to center.
        }

        if (runTime() >= timeoutMS) {
            stopVision();

            finished();
        }
    }

    private void stopVision() {
        robot.visionPortal.close();
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addData("Location", robot.teamProp.getLocation());
        engine.telemetry.addData("Saturation Left", robot.teamProp.getSaturationLeft());
        engine.telemetry.addData("Saturation Center", robot.teamProp.getSaturationCenter());
        engine.telemetry.addData("Saturation Right", robot.teamProp.getSaturationRight());
    }
}
