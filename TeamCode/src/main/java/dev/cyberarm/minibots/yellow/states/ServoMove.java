package dev.cyberarm.minibots.yellow.states;

import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.pizza.PizzaMinibot;
import dev.cyberarm.minibots.yellow.YellowMinibot;

public class ServoMove extends CyberarmState {
    final private YellowMinibot robot;
    final private String groupName, actionName;

    final private String servoName;
    final private double position;
    final private int timeoutMS;

    final private Servo servo;
    public ServoMove(YellowMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.groupName = groupName;
        this.actionName = actionName;

        this.servoName = robot.config.variable(groupName, actionName, "servoName").value();

        this.position = robot.config.variable(groupName, actionName, "position").value();

        this.timeoutMS  = robot.config.variable(groupName, actionName, "timeoutMS").value();

        this.servo = engine.hardwareMap.servo.get(servoName);
    }

    public ServoMove(YellowMinibot robot, String servoName, double position, int timeoutMS) {
        this.groupName = "";
        this.actionName = "";

        this.robot = robot;
        this.servoName = servoName;
        this.position = position;
        this.timeoutMS = timeoutMS;

        this.servo = engine.hardwareMap.servo.get(servoName);
    }

    @Override
    public void start() {
        this.servo.setPosition(this.position);
    }

    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            setHasFinished(true);
        }
    }
}
