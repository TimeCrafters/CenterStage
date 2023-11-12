package dev.cyberarm.minibots.yellow;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class YellowMinibot {
    public MotorEx leftFront, rightFront, leftBack, rightBack;
    public MotorGroup left, right;
    public IMU imu;
    public YellowMinibot(CyberarmEngine engine) {
        leftFront = new MotorEx(engine.hardwareMap, "leftFront");
        rightFront = new MotorEx(engine.hardwareMap, "rightFront");

        leftBack = new MotorEx(engine.hardwareMap, "leftBack");
        rightBack = new MotorEx(engine.hardwareMap, "rightBack");

        double distancePerTick = 1962.5; // 3.14 * 25mm * 25mm
        leftFront.setDistancePerPulse(distancePerTick);
        rightFront.setDistancePerPulse(distancePerTick);
        leftBack.setDistancePerPulse(distancePerTick);
        rightBack.setDistancePerPulse(distancePerTick);

        left = new MotorGroup(leftFront, leftBack);
        right = new MotorGroup(rightFront, rightBack);

        imu = engine.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);
    }
}
