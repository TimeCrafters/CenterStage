package dev.cyberarm.minibots.black.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.black.BlackMinibot;

public class BlackMinibotTeleOpState extends CyberarmState {
    final private BlackMinibot minibot;

    double maxPower = 0.2;
    double forward = 0;
    double right = 0;

    double leftPower = 0.0;
    double rightPower = 0.0;

    public BlackMinibotTeleOpState(BlackMinibot minibot) {
        this.minibot = minibot;
    }

    @Override
    public void exec() {
        forward = -engine.gamepad1.left_stick_y;
        right = engine.gamepad1.right_stick_x;

        leftPower = forward * maxPower;
        rightPower = forward * maxPower;

        // Simple TANK DRIVE
        if (forward > -0.01 && forward < 0.01) {
            if (right > 0) {
                leftPower = right * maxPower;
                rightPower = -right * maxPower;
            } else if (right < 0) {
                leftPower = -right * maxPower;
                rightPower = right * maxPower;
            }
        } else {
            if (right > 0) {
                leftPower += right * maxPower;
                rightPower -= right * maxPower;
            } else if (right < 0) {
                leftPower -= right * maxPower;
                rightPower += right * maxPower;
            }
        }

        minibot.leftDrive.motorEx.setPower(leftPower);
        minibot.RightDrive.motorEx.setPower(rightPower);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Cyberarm BLACK MINIBOT");
        engine.telemetry.addData("Forward", forward);
        engine.telemetry.addData("Right", right);
        engine.telemetry.addLine();
        engine.telemetry.addData("Max Power", maxPower);
        engine.telemetry.addData("Left Power", leftPower);
        engine.telemetry.addData("Right Power", rightPower);
    }
}
