package dev.cyberarm.minibots.black;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class BlackMinibot {
    final public MotorEx leftDrive, RightDrive;
    final private HardwareMap hardwareMap;

    public BlackMinibot() {
        hardwareMap = CyberarmEngine.instance.hardwareMap;

        leftDrive = new MotorEx(hardwareMap, "leftDrive");
        RightDrive = new MotorEx(hardwareMap, "rightDrive");
    }
}
