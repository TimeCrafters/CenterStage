package org.timecrafters.CenterStage.Common;

import org.timecrafters.Library.Robot;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class ProtoBotSodi extends Robot {

    public HardwareMap hardwareMap;
    public MotorEx flDrive, frDrive, blDrive, brDrive, liftMotor;
    public Servo fang, jaw, neck, shoulder, wrist, hand;
    public ProtoBotSodi robot;
    private String string;
    private CyberarmEngine engine;

    public ProtoBotSodi(String string) {
        this.engine = engine;
        this.string = string;
    }

    @Override
    public void setup() {System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        TimeCraftersConfiguration configuration = new TimeCraftersConfiguration("Robbie");

            //Motors
        //MOTORS
        frDrive = new MotorEx(hardwareMap, "frontRight");
        flDrive = new MotorEx(hardwareMap, "frontLeft");
        brDrive = new MotorEx(hardwareMap, "backRight");
        blDrive = new MotorEx(hardwareMap, "backLeft");

        robot.flDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.blDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.brDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Servos
        robot.jaw.setDirection(Servo.Direction.FORWARD);
        robot.fang.setDirection(Servo.Direction.FORWARD);
        robot.neck.setDirection(Servo.Direction.FORWARD);
        robot.shoulder.setDirection(Servo.Direction.FORWARD);
        robot.wrist.setDirection(Servo.Direction.FORWARD);
        robot.hand.setDirection(Servo.Direction.FORWARD);

//        fang = hardwareMap.servo.get("Fang");
//        jaw = hardwareMap.servo.get("Jaw");
//        neck = hardwareMap.servo.get("Neck");
//        shoulder = hardwareMap.servo.get("Shoulder");
//        wrist = hardwareMap.servo.get("Wrist");
//        hand = hardwareMap.servo.get("Hand");

    }
}
