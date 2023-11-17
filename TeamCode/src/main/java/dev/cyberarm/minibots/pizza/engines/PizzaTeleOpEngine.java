package dev.cyberarm.minibots.pizza.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.pizza.PizzaMinibot;
import dev.cyberarm.minibots.pizza.states.Pilot;

@TeleOp(name = "Cyberarm Pizza Teleop", group = "MINIBOT")
public class PizzaTeleOpEngine extends CyberarmEngine {
    PizzaMinibot robot;
    @Override
    public void setup() {
        robot = new PizzaMinibot(this);

        addState(new Pilot(robot));
    }

    @Override
    public void loop() {
        robot.standardTelemetry();
        robot.teleopTelemetry();

        super.loop();
    }
}
