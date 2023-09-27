package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.timecrafters.Library.Robot;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class PrototypeRobot extends Robot {

    private HardwareMap hardwareMap;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private RevIMU imu;

    private HDrive xDrive;
    private String string;
    private CyberarmEngine engine;

    public PrototypeRobot(String string) {
        this.string = string;
    }

    @Override
    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        //MOTORS
        frontRight = new MotorEx(hardwareMap, "frontRight");
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        backRight = new MotorEx(hardwareMap, "backRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");

        //IMU
        imu = hardwareMap.get(RevIMU.class, "imu");
        imu.init(new BNO055IMU.Parameters());

        // input motors exactly as shown below
        xDrive = new HDrive(frontLeft, frontRight,
                backLeft, backRight);

    }

    public void driveTrainTeleOp() {
        xDrive.driveFieldCentric(engine.gamepad1.left_stick_x, engine.gamepad1.left_stick_y, engine.gamepad1.right_stick_x, heading());
    }

    public double heading() {
        return imu.getHeading();
    }

}
