package dev.cyberarm.minibots.yellow.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.yellow.YellowMinibot;
import dev.cyberarm.minibots.yellow.states.Pilot;

@TeleOp(name = "Cyberarm Yellow Teleop", group = "MINIBOT")
public class YellowMinibotTeleOpEngine extends CyberarmEngine {
    public YellowMinibot robot;
    @Override
    public void setup() {
        robot = new YellowMinibot(this);

        addState(new Pilot(robot));
//        addState(new Move(robot, 10, 8, 0.25, 0.25));
    }
}
