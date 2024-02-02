package dev.cyberarm.minibots.red_crab.engines;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public abstract class RedCrabEngine extends CyberarmEngine {
    protected RedCrabMinibot robot;

    @Override
    public void loop() {
        Utilities.hubsClearBulkReadCache(hardwareMap);

        super.loop();

        robot.standardTelemetry();

        if (robot.autonomous) {
            if (runTime() < 20_000.0) { // KEEP CALM and CARRY ON
                robot.redLED.setState(robot.LED_OFF);
                robot.greenLED.setState(robot.LED_ON);
            } else if (runTime() < 25_000.0) { // RUNNING LOW ON TIME
                robot.redLED.setState(robot.LED_ON);
                robot.greenLED.setState(robot.LED_ON);
            } else { // 5 SECONDS LEFT!
                robot.redLED.setState(robot.LED_ON);
                robot.greenLED.setState(robot.LED_OFF);
            }
        } else {
            if (runTime() >= 90_000.0) { // LAUNCH DRONE and DO CHIN UP
                robot.redLED.setState(robot.LED_OFF);
                robot.greenLED.setState(robot.LED_ON);
            } else if (runTime() >= 80_000.0) { // GET READY
                robot.redLED.setState(robot.LED_ON);
                robot.greenLED.setState(robot.LED_ON);
            } else { // KEEP CALM and CARRY ON
                robot.redLED.setState(robot.LED_ON);
                robot.greenLED.setState(robot.LED_OFF);
            }
        }
    }

    @Override
    public void stop() {
        robot.shutdown();

        super.stop();
    }
}
