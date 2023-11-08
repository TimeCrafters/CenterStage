package org.timecrafters.CenterStage.TeleOp.States;

import org.timecrafters.CenterStage.Common.MiniYTeleOPBot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmState;

public class MiniYellowTeleOPv2 extends CyberarmState {
    private final MiniYTeleOPBot robot;
    private double flPower, frPower, blPower, brPower;
    private float lStickY, transitPercent = lStickY / 100;

    public MiniYellowTeleOPv2(MiniYTeleOPBot robot) {
        this.robot = robot;
    }


    @Override
    public void telemetry() {
    engine.telemetry.addData("FL Power", flPower);

    }

    @Override
    public void init() {

        flPower = 0;
        robot.flDrive.motor.setPower(flPower);

    }

    @Override
    public void exec() {

        transitPercent = -engine.gamepad1.left_stick_y * 100;
        flPower = lStickY / 100;
        robot.flDrive.motor.setPower(flPower);

    }
}
