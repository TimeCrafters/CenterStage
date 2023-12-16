package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class ClawArmMove extends CyberarmState {
    private final RedCrabMinibot robot;
    private final double power, targetAngle, toleranceAngle, gearRatio;
    private final int timeoutMS, motorTicks;
    public ClawArmMove(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.targetAngle = robot.config.variable(groupName, actionName, "angle").value();
        this.power = robot.config.variable(groupName, actionName, "power").value();
        this.toleranceAngle = robot.config.variable(groupName, actionName, "toleranceAngle").value();
        this.timeoutMS = robot.config.variable(groupName, actionName, "timeoutMS").value();

        this.motorTicks = RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION;
        this.gearRatio = RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO;
    }

    @Override
    public void start() {
        robot.clawArm.setPositionTolerance(Utilities.motorAngle(motorTicks, gearRatio, toleranceAngle));
        robot.clawArm.setTargetPosition(Utilities.motorAngle(motorTicks, gearRatio, targetAngle));

        engine.blackboardSet("clawArmPower", power);
    }

    @Override
    public void exec() {
        if (robot.clawArm.atTargetPosition() || runTime() >= timeoutMS) {
            this.finished();
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();
        engine.telemetry.addData("Motor Power", robot.clawArm.get());
        engine.telemetry.addData("Motor Position", robot.clawArm.getCurrentPosition());
        engine.telemetry.addData("Motor Angle", Utilities.motorAngle(motorTicks, gearRatio, robot.clawArm.getCurrentPosition()));
        engine.telemetry.addData("Motor Target Position", Utilities.motorAngle(motorTicks, gearRatio, targetAngle));
        engine.telemetry.addData("Motor Target Angle", targetAngle);
        engine.telemetry.addData("Timeout MS", timeoutMS);
        progressBar(20, runTime() / timeoutMS);
    }
}
