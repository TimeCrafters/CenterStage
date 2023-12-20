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
    private List<Recognition> recognitions = new ArrayList<>();

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

//        robot.tfPixel.setClippingMargins(0, 0, 0, 0);
//        robot.tfPixel.setMinResultConfidence((float) minConfidence);

        engine.blackboardSet("autonomousPath", path);
    }

    @Override
    public void exec() {
        if (engine.blackboardGetInt("autonomousPath") != 0) {
            this.finished();

            return;
        }

//        recognitions = robot.tfPixel.getRecognitions();
//        for (Recognition recognition : recognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2;
//
//            if (recognition.getLabel().equals("pixel")) {
//                engine.blackboardSet("autonomousPath", path);
//            }
//        }

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

//        engine.telemetry.addData("Saturation", robot.spikeMark.getSaturation());

//        engine.telemetry.addData("# Objects Detected", recognitions.size());
//
//        for(Recognition recognition : recognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2;
//
//            engine.telemetry.addLine();
//
//            engine.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            engine.telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            engine.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }
    }
}
