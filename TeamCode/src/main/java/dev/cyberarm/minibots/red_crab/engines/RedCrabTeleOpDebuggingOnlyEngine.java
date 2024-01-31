package dev.cyberarm.minibots.red_crab.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

@TeleOp(name = "Cyberarm Red Crab TeleOp DEBUGGING", group = "MINIBOT")
public class RedCrabTeleOpDebuggingOnlyEngine extends RedCrabEngine {
    @Override
    public void setup() {
        threadless();

        addState(new CyberarmState() {
            final RedCrabMinibot robot = new RedCrabMinibot(false);

            @Override
            public void exec() {
                double velocity = -engine.gamepad1.left_stick_y * Utilities.motorAngleToTicks(
                        RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION,
                        RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO,
                        RedCrabMinibot.CLAW_ARM_MAX_VELOCITY_DEGREES);

//                robot.clawArm.setVelocity(velocity);

                robot.clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.clawArm.setPower(-engine.gamepad1.left_stick_y * 0.5);
            }
        });
    }
}
