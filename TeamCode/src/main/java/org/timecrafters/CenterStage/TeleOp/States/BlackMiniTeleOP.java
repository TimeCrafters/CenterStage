package org.timecrafters.CenterStage.TeleOp.States;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.MiniBTeleOPBot;

import dev.cyberarm.engine.V2.CyberarmState;

public class BlackMiniTeleOP extends CyberarmState {
    private MiniBTeleOPBot robot;
    public BlackMiniTeleOP(MiniBTeleOPBot robot) {this.robot = robot;}


    private double rPower, lPower, servoPower;
    private boolean critTipPoint;

    public BlackMiniTeleOP() {}

    @Override
    public void telemetry() {
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

        if (Math.abs(engine.gamepad1.right_stick_y) > 0.1) {
            rPower = engine.gamepad1.right_stick_y;
            robot.rightDrive.motor.setPower(rPower);
        } else {
            rPower = 0;
            robot.rightDrive.motor.setPower(rPower);
        }

        if (Math.abs(engine.gamepad1.left_stick_y) > 0.1) {
            lPower = engine.gamepad1.left_stick_y;
            robot.leftDrive.motor.setPower(lPower);
        } else {
            lPower = 0;
            robot.leftDrive.motor.setPower(lPower);
        }

    }
}
