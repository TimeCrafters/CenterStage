package org.timecrafters.CenterStage.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class SodiPizzaMinibotObject extends Robot {

    public HardwareMap hardwareMap;
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public Servo shoulder, gripper;
    public IMU imu;
    private String string;

    public int readyToTurn;

    public static double GRIPPER_CLOSED = 0.333; // ~90 degrees
    public static double GRIPPER_OPEN = 0.75; // ~205 degrees

    public static double ARM_COLLECT = 0.0; // ~? degrees
    public static double ARM_PRECOLLECT = 0.05; // ~? degrees
    public static double ARM_DELIVER = 0.28; // ~? degrees
    public static double ARM_STOW = 0.72; // ~? degrees
    public static double ARM_HOVER_5_STACK = 0.10;
    public static double ARM_HOVER_4_STACK = 0.08;
    public static double ARM_HOVER_3_STACK = 0.07;
    public static double ARM_HOVER_2_STACK = 0.06;

    private CyberarmEngine engine;
    public TimeCraftersConfiguration configuration;

    public SodiPizzaMinibotObject() {}

    @Override
    public void setup() {

        this.engine = CyberarmEngine.instance;
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;

        //Motor defining
        leftFront = engine.hardwareMap.dcMotor.get("leftFront");
        rightFront = engine.hardwareMap.dcMotor.get("rightFront");
        leftBack = engine.hardwareMap.dcMotor.get("leftBack");
        rightBack = engine.hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servo Defining
        shoulder = engine.hardwareMap.servo.get("arm");
        gripper = engine.hardwareMap.servo.get("gripper");

        //readyToTurn
    }
}
