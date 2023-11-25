package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.CenterStage.Autonomous.States.SodiPizzaAutoArmState;
import org.timecrafters.CenterStage.Autonomous.States.SodiPizzaAutoFirstDriveState;
import org.timecrafters.CenterStage.Autonomous.States.SodiPizzaAutoTurnState;
import org.timecrafters.CenterStage.Autonomous.States.SodiPizzaWheelTest;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Sodi's Pizza Box Bot Auto", group = "")
public class SodiPizzaAutoRedRightEngine extends CyberarmEngine {
    @Override
    public void setup() {
        addState(new SodiPizzaAutoFirstDriveState());
        addState(new SodiPizzaAutoTurnState());
//        addState(new SodiPizzaAutoArmState());
//        addState(new SodiPizzaWheelTest());
    }
}
