package org.timecrafters.CenterStage.Autonomous.Engines;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.CenterStage.Common.ProtoBotSodi;
import org.timecrafters.CenterStage.Autonomous.States.ProtoBotStateSodi;
import org.timecrafters.CenterStage.Autonomous.States.ProtoBotStateSodi;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Rigel", group = "Robot")
public class ProtoBotEngineSodi extends CyberarmEngine {
    private ProtoBotSodi robot;
    @Override
    public void setup() {
        this.robot = new ProtoBotSodi();
        this.robot.setup();

        addState(new ProtoBotStateSodi());
    }
}
