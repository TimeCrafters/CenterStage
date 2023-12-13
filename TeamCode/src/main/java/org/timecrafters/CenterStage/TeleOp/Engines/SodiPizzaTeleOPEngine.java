package org.timecrafters.CenterStage.TeleOp.Engines;

import org.timecrafters.CenterStage.TeleOp.States.SodiPizzaTeleOPState;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class SodiPizzaTeleOPEngine extends CyberarmEngine {
    @Override
    public void setup() {
        addState(new SodiPizzaTeleOPState());
    }
}
