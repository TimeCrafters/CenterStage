package org.timecrafters.CenterStage.Common;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

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
    public Servo grabJaw, grabElbow, grabShoulder, dropShoulder, dropElbow, dropJaw;
    private String string;
    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;


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
        frDrive = new MotorEx(hardwareMap, "FrontRight");
        flDrive = new MotorEx(hardwareMap, "FrontLeft");
        brDrive = new MotorEx(hardwareMap, "BackRight");
        blDrive = new MotorEx(hardwareMap, "BackLeft");
        liftMotor = new MotorEx(hardwareMap, "Lift");

        flDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flDrive.motor.setDirection(FORWARD);
        frDrive.motor.setDirection(REVERSE);
        blDrive.motor.setDirection(FORWARD);
        brDrive.motor.setDirection(REVERSE);

        //Servos
        grabJaw = hardwareMap.servo.get("GrabJaw");
        grabElbow = hardwareMap.servo.get("GrabElbow");
        grabShoulder = hardwareMap.servo.get("GrabShoulder");
        dropShoulder = hardwareMap.servo.get("DropShoulder");
        dropElbow = hardwareMap.servo.get("DropElbow");
        dropJaw = hardwareMap.servo.get("DropJaw");
     
        grabElbow.setDirection(Servo.Direction.FORWARD);
        grabJaw.setDirection(Servo.Direction.FORWARD);
        grabShoulder.setDirection(Servo.Direction.FORWARD);
        dropShoulder.setDirection(Servo.Direction.FORWARD);
        dropElbow.setDirection(Servo.Direction.FORWARD);
        dropJaw.setDirection(Servo.Direction.FORWARD);


    }
}
