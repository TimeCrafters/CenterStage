package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.CenterStage.Autonomous.SodiStates.SodiPizzaAutoFirstDriveState;
import org.timecrafters.CenterStage.Autonomous.SodiStates.SodiPizzaAutoTurnState;
import org.timecrafters.CenterStage.Autonomous.SodiStates.SodiPizzaAutoSecDriveState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Sodi's Pizza Box Bot Auto", group = "")
public class SodiPizzaAutoRedRightEngine extends CyberarmEngine {

    @Override
    public void setup() {
        blackboardSet("readyToTurn", 0);
        addState(new SodiPizzaAutoFirstDriveState());
        addState(new SodiPizzaAutoTurnState());
        addState(new SodiPizzaAutoSecDriveState());

    }
}
