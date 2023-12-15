package dev.cyberarm.minibots.red_crab.states;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class PathSelector extends CyberarmState {
    private final RedCrabMinibot robot;
    private final int timeoutMS;
    private final int fallbackPath;
    private List<Recognition> recognitions = new ArrayList<>();

    public PathSelector(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeoutMS").value();
        this.fallbackPath = robot.config.variable(groupName, actionName, "fallbackPath").value();
    }

    @Override
    public void init() {
        robot.tfod.setClippingMargins(0, 0, 0, 0);
        robot.tfod.setMinResultConfidence(0.8f);

        engine.blackboardSet("autonomousPath", fallbackPath);
    }

    @Override
    public void exec() {
        recognitions = robot.tfod.getRecognitions();
        for (Recognition recognition : recognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2;

            if (recognition.getLabel().equals("pixel")) {
                if (x < 120) {
                    engine.blackboardSet("autonomousPath", RedCrabMinibot.Path.LEFT.ordinal());
                } else if (x >= 120 && x < 240) {
                    engine.blackboardSet("autonomousPath", RedCrabMinibot.Path.CENTER.ordinal());
                } else {
                    engine.blackboardSet("autonomousPath", RedCrabMinibot.Path.RIGHT.ordinal());
                }
            }
        }

        if (runTime() >= timeoutMS) {
            robot.visionPortal.close();
            robot.tfod.shutdown();
            finished();
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addData("# Objects Detected", recognitions.size());

        for(Recognition recognition : recognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2;

            engine.telemetry.addLine();

            engine.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            engine.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            engine.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}
