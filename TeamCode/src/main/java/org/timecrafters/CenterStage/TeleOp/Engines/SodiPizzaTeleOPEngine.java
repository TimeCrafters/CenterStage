package org.timecrafters.CenterStage.TeleOp.Engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.TeleOp.States.SodiPizzaTeleOPState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Sodi Pizza Box Bot TeleOP", group = "")
public class SodiPizzaTeleOPEngine extends CyberarmEngine {
    @Override
    public void setup() {
        addState(new SodiPizzaTeleOPState());
    }
}
