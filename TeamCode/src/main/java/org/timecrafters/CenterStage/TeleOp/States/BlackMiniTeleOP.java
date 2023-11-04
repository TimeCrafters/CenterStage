package org.timecrafters.CenterStage.TeleOp.States;

import org.timecrafters.CenterStage.Common.MiniBTeleOPBot;

import dev.cyberarm.engine.V2.CyberarmState;

public class BlackMiniTeleOP extends CyberarmState {
    private MiniBTeleOPBot robot;
    public BlackMiniTeleOP(MiniBTeleOPBot robot) {this.robot = robot;}


    private double rPower, lPower;

    public BlackMiniTeleOP() {
    }

    @Override
    public void init() {
        rPower = 0;
        lPower = 0;
        robot.rightDrive.motor.setPower(rPower);
        robot.leftDrive.motor.setPower(lPower);

    }
    @Override
    public void exec() {

        if (engine.gamepad1.right_trigger > 0.1) {
            rPower = engine.gamepad1.right_trigger;
            robot.rightDrive.motor.setPower(rPower);
        }

        if (engine.gamepad1.left_trigger > 0.1) {
            lPower = engine.gamepad1.left_trigger;
            robot.leftDrive.motor.setPower(lPower);
        }



    }
}
