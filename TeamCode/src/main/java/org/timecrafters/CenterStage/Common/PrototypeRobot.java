package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;


public class PrototypeRobot extends Robot {

//    public double integralSum = 0;
//    double Kp = 0;
//    double Ki = 0;
//    double Kd = 0;
//    public double lastError = 0;
    public double PIDrx;
    public double targetHeading;
    public boolean headingLock = false;
    public double backDropLock = Math.toRadians(90);
    ElapsedTime timer = new ElapsedTime();
    public int armPosition = 0;
    public boolean stateFinished;
    public long startOfSequencerTime;
    public int oldArmPosition = 0;
    public long waitTime;
    public double servoWaitTime;
    public double servoSecPerDeg = 0.14/60;
    public float DEPOSITOR_SHOULDER_IN;
    public float DEPOSITOR_SHOULDER_OUT;
    public float DEPOSITOR_ELBOW_IN;
    public float DEPOSITOR_ELBOW_OUT;
    public float COLLECTOR_SHOULDER_IN;
    public float COLLECTOR_SHOULDER_PASSIVE;
    public float COLLECTOR_SHOULDER_OUT;
    public float COLLECTOR_ELBOW_IN;
    public float COLLECTOR_ELBOW_PASSIVE;
    public float COLLECTOR_ELBOW_OUT;
    public float lastSetPosShoulder;
    public float lastSetPosElbow;
    public float currentSetPosShoulder;
    public float currentSetPosElbow;
    private HardwareMap hardwareMap;
    public DcMotor frontLeft, frontRight, backLeft, backRight, lift;
    public DcMotor odometerR, odometerL, odometerA;
    public IMU imu;
    public Servo depositorShoulder, depositorElbow, collectorShoulder, collectorElbow, depositor, collector;
    private HDrive xDrive;
    private String string;
    private double drivePower = 1;

    public double xMultiplier = 1;
    public double yMultiplier = 1;
    public double positionX;
    public double positionY;
    public double positionH;

    // robot geometry constants for odometry -----------------------------------------------------------------------------------------------
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;
    public int oldRightPosition = 0;
    public int oldLeftPosition = 0;
    public int oldAuxPosition = 0;
    public double globalPositionX;
    public double globalPositionY;
    public double globalPositionH;
    public double localPositionX;
    public double localPositionY;
    public double localPositionH;

    public final static double L = 23.425; // distance between left and right encoder in cm
    final static double B = 10; // distance between the midpoint of the left and right encoder from the auxillary encoder in cm
    public final static double R = 4.5; // wheel radius in cm
    final static double N = 8192; // encoder ticks per revolution (REV encoder)

    public final double MaxVelocityForward = 40;
    public final double MaxStrafeVelocity = 34;
    public final double MaxRotationalVelocity = 20;
    private boolean lbsVar1;
    private boolean lbsVar2;
    private boolean rbsVar2;
    public float depositorPos;
    public float collectorPos;
    public double rx;




    public final double cm_per_tick = (2 * Math.PI * R) / N;
    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;

    public PrototypeRobot(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void initConstants(){
        DEPOSITOR_SHOULDER_IN = configuration.variable("Robot", "Tuning", "DEPOSITOR_SHOULDER_IN").value();
        DEPOSITOR_SHOULDER_OUT = configuration.variable("Robot", "Tuning", "DEPOSITOR_SHOULDER_OUT").value();
        DEPOSITOR_ELBOW_IN = configuration.variable("Robot", "Tuning", "DEPOSITOR_ELBOW_IN").value();
        DEPOSITOR_ELBOW_OUT = configuration.variable("Robot", "Tuning", "DEPOSITOR_ELBOW_OUT").value();
        COLLECTOR_SHOULDER_IN = configuration.variable("Robot", "Tuning", "COLLECTOR_SHOULDER_IN").value();
        COLLECTOR_SHOULDER_PASSIVE = configuration.variable("Robot", "Tuning", "COLLECTOR_SHOULDER_PASSIVE").value();
        COLLECTOR_SHOULDER_OUT = configuration.variable("Robot", "Tuning", "COLLECTOR_SHOULDER_OUT").value();
        COLLECTOR_ELBOW_IN = configuration.variable("Robot", "Tuning", "COLLECTOR_ELBOW_IN").value();
        COLLECTOR_ELBOW_PASSIVE = configuration.variable("Robot", "Tuning", "COLLECTOR_ELBOW_PASSIVE").value();
        COLLECTOR_ELBOW_OUT = configuration.variable("Robot", "Tuning", "COLLECTOR_ELBOW_OUT").value();
        }

    @Override
    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        imu = engine.hardwareMap.get(IMU.class, "imu");

        //MOTORS
        frontRight = engine.hardwareMap.dcMotor.get("frontRight");
        frontLeft = engine.hardwareMap.dcMotor.get("frontLeft");
        backRight = engine.hardwareMap.dcMotor.get("backRight");
        backLeft = engine.hardwareMap.dcMotor.get("backLeft");
        lift = engine.hardwareMap.dcMotor.get("Lift");
        odometerR = engine.hardwareMap.dcMotor.get("odometerR");

        configuration = new TimeCraftersConfiguration("Blue Crab");

        initConstants();

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);




        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        //SERVO
        depositorShoulder = hardwareMap.servo.get("depositor_shoulder");
        depositorElbow = hardwareMap.servo.get("depositor_elbow");
        collectorShoulder = hardwareMap.servo.get("collector_shoulder");
        collectorElbow = hardwareMap.servo.get("collector_elbow");
        depositor = hardwareMap.servo.get("depositor");
        collector = hardwareMap.servo.get("collector");

        depositorShoulder.setPosition(DEPOSITOR_SHOULDER_IN);
        depositorElbow.setPosition(DEPOSITOR_ELBOW_IN);
        collectorElbow.setPosition(COLLECTOR_ELBOW_IN);
        collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN);
        collector.setPosition(0F);



    }

    public void driveTrainTeleOp() {
        boolean lbs1 = engine.gamepad1.left_stick_button;
        if (lbs1 && !lbsVar1) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbsVar1 = lbs1;

        double y = -engine.gamepad1.left_stick_y;
        double x = engine.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

        if (headingLock){
            ;
        } else if (headingLock == false){
            rx = engine.gamepad1.right_stick_x;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // angle math to make things field oriented
        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // joystick math to find the approximate power across each wheel for a movement
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY + rotX - rx) / denominator;
        double backRightPower = (rotY - rotX - rx) / denominator;

        // setting each power determined previously from the math above
        // as well as multiplying it by a drive power that can be changed.
        backLeft.setPower(backLeftPower * drivePower);
        backRight.setPower(backRightPower * drivePower);
        frontLeft.setPower(frontLeftPower * drivePower);
        frontRight.setPower(frontRightPower * drivePower);
    }

    public void ShoulderServoWaitTime(){

        servoWaitTime = servoSecPerDeg * (Math.abs(lastSetPosShoulder - currentSetPosShoulder));

    }

    public void ElbowServoWaitTime(){

        servoWaitTime = 1000 * (servoSecPerDeg * (Math.abs(lastSetPosElbow - currentSetPosElbow)));

    }
    public void OdometryLocalizer(){

        if (Math.toDegrees(positionH) > 360){
            positionH -= 360;
        }

        globalPositionX = localPositionX;
        globalPositionY = localPositionY;
        globalPositionH = localPositionH;


        // update positions
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -odometerR.getCurrentPosition();
        currentLeftPosition = -odometerL.getCurrentPosition();
        currentAuxPosition = odometerA.getCurrentPosition();

        int dnl1 = currentLeftPosition - oldLeftPosition;
        int dnr2 = currentRightPosition - oldRightPosition;
        int dna3 = currentAuxPosition - oldAuxPosition;

        // the robot has turned and moved a tiny bit between two measurements
        double dtheta = cm_per_tick * (dnr2 - dnl1) / L;
        double dx = cm_per_tick * (dnl1 + dnr2) / 2.0;
        double dy = cm_per_tick * (dna3 - (dnr2 - dnl1) * B / L);

        // the small movement of the bot gets added to the field coordinates
        double theta = positionH + (dtheta / 2.0);
        positionX += (dx * Math.cos(theta) - dy * Math.sin(theta)) * xMultiplier;
        positionY += (dx * Math.sin(theta) + dy * Math.cos(theta)) * yMultiplier;
        positionH += dtheta;

    }

//    public double PIDControl(double reference, double current){
//        double error = reference - current;
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//
//        timer.reset();
//
//        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
//        return output;
//    }

    public void CollectorToggle(){
        boolean lbs2 = engine.gamepad2.left_stick_button;
        if (lbs2 && !lbsVar2) {
            if (collectorPos == 1F) {
                collectorPos = 0F;
            } else {
                collectorPos = 1F;
            }
        }
        lbsVar2 = lbs2;
    }
    public void DepositorToggle(){
        boolean rbs2 = engine.gamepad2.right_stick_button;
        if (rbs2 && !rbsVar2) {
            if (depositorPos == 0.6F) {
                depositorPos = 0F;
            } else {
                depositorPos = 0.6F;
            }
        }
        rbsVar2 = rbs2;
    }
    public void ArmSequences(){
        switch (armPosition) {
            case 0: // ----------------------------------------------------------------------------------------------- drive to transfer pos
                switch (oldArmPosition) {
                    case 0:
                        // transfer
                    case 1:
                        // driving
                        collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 800) { // wait to move till time is met
                            collectorElbow.setPosition(COLLECTOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
                                oldArmPosition = 0;
                            }
                        }
                    case 2:
                        // collect
                        collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            collectorElbow.setPosition(COLLECTOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
                                oldArmPosition = 0;
                            }
                        }
                    case 3:
                        // deposit
                        depositorShoulder.setPosition(DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 800) { // wait to move till time is met
                            depositorElbow.setPosition(DEPOSITOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1600) {
                                collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                                if (System.currentTimeMillis() - startOfSequencerTime >= 2300) { // wait to move till time is met
                                    collectorElbow.setPosition(COLLECTOR_ELBOW_IN);
                                    if (System.currentTimeMillis() - startOfSequencerTime >= 3100) {
                                        oldArmPosition = 0;
                                    }
                                }
                            }
                        }
                }

//          case 1:// ----------------------------------------------------------------------------------------------- drive to driving pos
//                switch (oldArmPosition) {
//                    case 0:
//                        // transfer
//                        collectorShoulder.setPosition(COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_SHOULDER_PASSIVE);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
//                                oldArmPosition = 1;
//                            }
//                        }
//                        break;
//                    case 1:
//                        // drive pos
//                        break;
//                    case 2:
//                        // collect
//                        collectorShoulder.setPosition(COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 600) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_SHOULDER_PASSIVE);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 2100) {
//                                oldArmPosition = 1;
//                            }
//                        }
//                        break;
//                    case 3:
//                        // deposit
//                        depositorShoulder.setPosition(DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 800) { // wait to move till time is met
//                            depositorElbow.setPosition(DEPOSITOR_ELBOW_IN);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 1600) {
//                                collectorShoulder.setPosition(COLLECTOR_SHOULDER_PASSIVE); // drive the shoulder to the transfer position
//                                if (System.currentTimeMillis() - startOfSequencerTime >= 2300) { // wait to move till time is met
//                                    collectorElbow.setPosition(COLLECTOR_ELBOW_PASSIVE);
//                                    if (System.currentTimeMillis() - startOfSequencerTime >= 3100) {
//                                        oldArmPosition = 1;
//                                    }
//                                }
//                            }
//                        }
//                        break;
//                }
//                break;
//
//            case 2:// ----------------------------------------------------------------------------------------------- drive to collect pos
//                switch (oldArmPosition) {
//                    case 0:
//                        // transfer
//                        collectorShoulder.setPosition(COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_SHOULDER_OUT);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
//                                oldArmPosition = 2;
//                            }
//                        }
//                        break;
//                    case 1:
//                        // driving
//                        collectorShoulder.setPosition(COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_SHOULDER_OUT);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
//                                oldArmPosition = 2;
//                            }
//                        }
//                        break;
//                    case 2:
//                        // collect
//                        collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_ELBOW_IN);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
//                                oldArmPosition = 2;
//                            }
//                        }
//                        break;
//                    case 3:
//                        // deposit
//                        depositorShoulder.setPosition(DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 100) { // wait to move till time is met
//                            depositorElbow.setPosition(DEPOSITOR_ELBOW_IN);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 100) {
//                                if (System.currentTimeMillis() - startOfSequencerTime >= 100) { // wait to move till time is met
//                                    collectorElbow.setPosition(COLLECTOR_ELBOW_OUT);
//                                    if (System.currentTimeMillis() - startOfSequencerTime >= 100) {
//                                        oldArmPosition = 2;
//                                    }
//                                }
//                            }
//                        }
//                        break;
//                }
//                break;

//            case 3:// ----------------------------------------------------------------------------------------------- drive to deposit pos
//                switch (oldArmPosition) {
//                    case 0:
//                        // transfer
//                        collectorShoulder.setPosition(COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_SHOULDER_PASSIVE);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
//                                oldArmPosition = 3;
//                            }
//                        }
//                        break;
//                    case 1:
//                        // driving
//                        collectorShoulder.setPosition(COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 700) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_SHOULDER_PASSIVE);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
//                                oldArmPosition = 3;
//                            }
//                        }
//                        break;
//                    case 2:
//                        // collect
//                        collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
//                            collectorElbow.setPosition(COLLECTOR_ELBOW_IN);
//                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
//                                oldArmPosition = 3;
//                            }
//                        }
//                        break;
//                    case 3:
//                        // deposit
//                        break;
//                }
//                break;

        }
    }
}