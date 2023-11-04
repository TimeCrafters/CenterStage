package dev.cyberarm.minibots.black.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.cyberarm.minibots.black.BlackMinibot;
import dev.cyberarm.minibots.black.states.BlackMinibotTeleOpState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Cyberarm Black Minibot", group = "MINIBOT")
public class BlackMinibotTeleOpEngine extends CyberarmEngine {
    private BlackMinibot minibot;
    @Override
    public void setup() {
        minibot = new BlackMinibot();

        addState(new BlackMinibotTeleOpState(minibot));
    }
}
