package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.Library.Robot;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class PrototypeRobot extends Robot {

    private HardwareMap hardwareMap;
    public MotorEx frontLeft, frontRight, backLeft, backRight;
    private RevIMU imu;
    public Servo depositorFlip, depositor;
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

        frontRight.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.motor.setDirection(DcMotorSimple.Direction.FORWARD);


        frontRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //IMU
        imu = new RevIMU(hardwareMap, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.init(parameters);

        //SERVO
        depositorFlip = hardwareMap.servo.get("depositorFlip");
        depositor = hardwareMap.servo.get("depositor");

        // input motors exactly as shown below
        xDrive = new HDrive(frontLeft, frontRight,
                            backLeft, backRight);

    }

    public void driveTrainTeleOp() {
        xDrive.driveRobotCentric(-engine.gamepad1.left_stick_x, engine.gamepad1.left_stick_y, -engine.gamepad1.right_stick_x);
    }

    public double heading() {
        return imu.getHeading();
    }

}
