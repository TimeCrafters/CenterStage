package org.timecrafters.diagnostics.rev_hub_test_suite;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.Library.Robot;

import java.util.ArrayList;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class RevHubTestSuiteRobot extends Robot {
    final String TAG = "RevTestSuite|Robot";

    private HardwareMap hardwareMap;
    public boolean testingControlHub = true;
    public ArrayList<MotorEx> controlHubMotors = new ArrayList<>(), expansionHubMotors = new ArrayList<>();
    public ArrayList<ServoEx> controlHubServos = new ArrayList<>(), expansionHubServos = new ArrayList<>();
    public ArrayList<AnalogInput> controlHubAnalogSensors = new ArrayList<>(), expansionHubAnalogSensors = new ArrayList<>();
    public ArrayList<DigitalChannel> controlHubDigitalSensors = new ArrayList<>(), expansionHubDigitalSensors = new ArrayList<>();
    public ArrayList<Rev2mDistanceSensor> controlHubI2cSensors = new ArrayList<>(), expansionHubI2cSensors = new ArrayList<>();

    public ArrayList<LynxModule> lynxModules = new ArrayList<>();
    @Override
    public void setup() {
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;

        this.lynxModules = new ArrayList<>(hardwareMap.getAll(LynxModule.class));

        /* ------------------------------------------------ Control Hub Devices ------------------------------------------------ */
        // MOTORS
        controlHubMotors.add(new MotorEx(hardwareMap, "c_motor_0"));
        controlHubMotors.add(new MotorEx(hardwareMap, "c_motor_1"));
        controlHubMotors.add(new MotorEx(hardwareMap, "c_motor_2"));
        controlHubMotors.add(new MotorEx(hardwareMap, "c_motor_3"));

//        // SERVOS
//        controlHubServos.add(new SimpleServo(hardwareMap, "c_servo_0", 0.0, 180.0, AngleUnit.DEGREES));
//        controlHubServos.add(new SimpleServo(hardwareMap, "c_servo_1", 0.0, 180.0, AngleUnit.DEGREES));
//        controlHubServos.add(new SimpleServo(hardwareMap, "c_servo_2", 0.0, 180.0, AngleUnit.DEGREES));
//        controlHubServos.add(new SimpleServo(hardwareMap, "c_servo_3", 0.0, 180.0, AngleUnit.DEGREES));
//        controlHubServos.add(new SimpleServo(hardwareMap, "c_servo_4", 0.0, 180.0, AngleUnit.DEGREES));
//        controlHubServos.add(new SimpleServo(hardwareMap, "c_servo_5", 0.0, 180.0, AngleUnit.DEGREES));
//
//        // ANALOG SENSORS
//        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_0"));
//        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_1"));
//        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_2"));
//        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_3"));
//
//        // DIGITAL SENSORS
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_0"));
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_1"));
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_2"));
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_3"));
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_4"));
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_5"));
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_6"));
//        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_7"));
//
//        // I2C SENSORS
//        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_0"));
//        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_1"));
//        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_2"));
//        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_3"));
//
//        /* ------------------------------------------------ Expansion Hub Devices ------------------------------------------------ */
//        // MOTORS
//        expansionHubMotors.add(new MotorEx(hardwareMap, "x_motor_0"));
//        expansionHubMotors.add(new MotorEx(hardwareMap, "x_motor_1"));
//        expansionHubMotors.add(new MotorEx(hardwareMap, "x_motor_2"));
//        expansionHubMotors.add(new MotorEx(hardwareMap, "x_motor_3"));
//
//        // SERVOS
//        expansionHubServos.add(new SimpleServo(hardwareMap, "x_servo_0", 0.0, 180.0, AngleUnit.DEGREES));
//        expansionHubServos.add(new SimpleServo(hardwareMap, "x_servo_1", 0.0, 180.0, AngleUnit.DEGREES));
//        expansionHubServos.add(new SimpleServo(hardwareMap, "x_servo_2", 0.0, 180.0, AngleUnit.DEGREES));
//        expansionHubServos.add(new SimpleServo(hardwareMap, "x_servo_3", 0.0, 180.0, AngleUnit.DEGREES));
//        expansionHubServos.add(new SimpleServo(hardwareMap, "x_servo_4", 0.0, 180.0, AngleUnit.DEGREES));
//        expansionHubServos.add(new SimpleServo(hardwareMap, "x_servo_5", 0.0, 180.0, AngleUnit.DEGREES));
//
//        // ANALOG SENSORS
//        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_0"));
//        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_1"));
//        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_2"));
//        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_3"));
//
//        // DIGITAL SENSORS
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_0"));
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_1"));
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_2"));
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_3"));
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_4"));
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_5"));
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_6"));
//        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_7"));
//
//        // I2C SENSORS
//        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_0"));
//        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_1"));
//        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_2"));
//        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_3"));
//
//        /* ------------------------------------------------ Hub Sensor Reading Optimization ------------------------------------------------ */
//        for (LynxModule hub : lynxModules) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        /* ------------------------------------------------ Motor Configuration ------------------------------------------------ */
        configureMotors();
    }

    private void configureMotors() {
        for(MotorEx motor : controlHubMotors) {
            motor.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.stopAndResetEncoder();
        }

        for(MotorEx motor : expansionHubMotors) {
            motor.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.stopAndResetEncoder();
        }
    }

    protected void clearStaleData() {
//        for (LynxModule hub : lynxModules) {
//            hub.clearBulkCache();
//        }
    }
}
