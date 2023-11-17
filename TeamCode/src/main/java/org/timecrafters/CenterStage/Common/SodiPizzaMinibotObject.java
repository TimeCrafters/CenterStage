package org.timecrafters.CenterStage.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class SodiPizzaMinibotObject extends Robot {

    public HardwareMap hardwareMap;
    public DcMotor flDrive, frDrive, blDrive, brDrive;
    public Servo shoulder, hand;
    public IMU imu;
    private String string;

    private CyberarmEngine engine;
    public TimeCraftersConfiguration configuration;

    public SodiPizzaMinibotObject() {}

    @Override
    public void setup() {

        this.engine = CyberarmEngine.instance;
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;

        //Motor defining
        flDrive = engine.hardwareMap.dcMotor.get("FL Drive");
        frDrive = engine.hardwareMap.dcMotor.get("FR Drive");
        blDrive = engine.hardwareMap.dcMotor.get("BL Drive");
        brDrive = engine.hardwareMap.dcMotor.get("BR Drive");


    }
}
