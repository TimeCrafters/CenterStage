package dev.cyberarm.minibots.red_crab.engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.minibots.red_crab.RedCrabMinibot;
import dev.cyberarm.minibots.red_crab.states.ClawArmTask;
import dev.cyberarm.minibots.red_crab.states.LocalizerTask;

@Autonomous(name = "Cyberarm Red Crab Auto DEBUGGING", group = "MINIBOT", preselectTeleOp = "Cyberarm Red Crab TeleOp")
public class RedCrabAutonomousDebuggingOnlyEngine extends RedCrabEngine {
    @Override
    public void setup() {
        robot = new RedCrabMinibot(true);
        addTask(new ClawArmTask(robot));
        addTask(new LocalizerTask(robot));

        setupFromConfig(
                new TimeCraftersConfiguration("cyberarm_RedCrab"),
                "dev.cyberarm.minibots.red_crab.states",
                robot,
                RedCrabMinibot.class,
                "Autonomous_Z_DEBUG"
        );
    }
}