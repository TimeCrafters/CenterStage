package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.CenterStage.Autonomous.States.SodiPizzaAutoArmState;
import org.timecrafters.CenterStage.Autonomous.States.SodiPizzaAutoDriveState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Sodi's Pizza Box Bot Auto", group = "")
public class SodiPizzaAutoRedRightEngine extends CyberarmEngine {
    @Override
    public void setup() {
        addState(new SodiPizzaAutoDriveState());
        addState(new SodiPizzaAutoArmState());
    }
}
