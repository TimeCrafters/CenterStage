package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class ServoMove extends CyberarmState {
    private final RedCrabMinibot robot;
    private final Servo servo;
    private final double startingPosition, targetPosition, lerpMS;
    private final int timeoutMS;
    private final boolean lerp;
    private final String servoName;
    public ServoMove(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.servoName = robot.config.variable(groupName, actionName, "servoName").value();
        this.targetPosition = robot.config.variable(groupName, actionName, "position").value();
        this.lerp = robot.config.variable(groupName, actionName, "lerp").value();
        this.lerpMS = robot.config.variable(groupName, actionName, "lerpMS").value();
        this.timeoutMS = robot.config.variable(groupName, actionName, "timeoutMS").value();

        this.servo = engine.hardwareMap.servo.get(servoName);
        this.startingPosition = this.servo.getPosition();
    }

    @Override
    public void exec() {
        if (lerp) {
            servo.setPosition(
                    Utilities.lerp(
                            startingPosition,
                            targetPosition,
                            Range.clip(runTime() / lerpMS, 0.0, 1.0))
            );
        } else {
            servo.setPosition(targetPosition);
        }

        if (runTime() >= timeoutMS) {
            servo.setPosition(targetPosition);
            this.finished();
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();
        engine.telemetry.addData("Servo", servoName);
        engine.telemetry.addData("Servo Position", servo.getPosition());
        engine.telemetry.addData("Servo Starting Position", startingPosition);
        engine.telemetry.addData("Servo Target Position", targetPosition);
        engine.telemetry.addData("lerp", lerp);
        engine.telemetry.addData("lerp MS", lerpMS);
        progressBar(20, runTime() / lerpMS);
        engine.telemetry.addLine();
        engine.telemetry.addData("Timeout MS", timeoutMS);
        progressBar(20, runTime() / timeoutMS);
    }
}
