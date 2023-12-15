package dev.cyberarm.minibots.red_crab.engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

@Autonomous(name = "Cyberarm Red Crab RED AUDIENCE", group = "MINIBOT", preselectTeleOp = "Cyberarm Red Crab TeleOp")
public class RedCrabAutonomousRedAudienceEngine extends CyberarmEngine {
    @Override
    public void setup() {
        setupFromConfig(
                new TimeCraftersConfiguration("cyberarm_RedCrab"),
                "dev.cyberarm.minibots.red_crab.states",
                new RedCrabMinibot(true),
                RedCrabMinibot.class,
                "Autonomous_RED_Audience"
        );
    }
}
