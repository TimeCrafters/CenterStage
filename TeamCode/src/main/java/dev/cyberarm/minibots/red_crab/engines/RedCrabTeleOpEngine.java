package dev.cyberarm.minibots.red_crab.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;
import dev.cyberarm.minibots.red_crab.states.ClawArmTask;
import dev.cyberarm.minibots.red_crab.states.LocalizerTask;
import dev.cyberarm.minibots.red_crab.states.Pilot;

@TeleOp(name = "Cyberarm Red Crab TeleOp", group = "MINIBOT")
public class RedCrabTeleOpEngine extends RedCrabEngine {
    @Override
    public void setup() {
        robot = new RedCrabMinibot(false);

        addTask(new ClawArmTask(robot));
        addTask(new LEDControllerTask(robot));
        addTask(new LocalizerTask(robot));

        addState(new Pilot(robot));
    }
}
