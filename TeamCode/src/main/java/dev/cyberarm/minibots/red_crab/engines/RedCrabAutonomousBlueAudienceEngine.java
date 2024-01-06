package dev.cyberarm.minibots.red_crab.engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;
import dev.cyberarm.minibots.red_crab.states.ClawArmTask;

@Autonomous(name = "Cyberarm Red Crab BLUE AUDIENCE", group = "MINIBOT", preselectTeleOp = "Cyberarm Red Crab TeleOp")
public class RedCrabAutonomousBlueAudienceEngine extends RedCrabAutonomousEngine {
    @Override
    public void setup() {
        robot = new RedCrabMinibot(true);
        addTask(new ClawArmTask(robot));

        setupFromConfig(
                new TimeCraftersConfiguration("cyberarm_RedCrab"),
                "dev.cyberarm.minibots.red_crab.states",
                robot,
                RedCrabMinibot.class,
                "Autonomous_BLUE_Audience"
        );
    }
}
