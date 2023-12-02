package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.drivebase.HDrive;
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


public class XDrivetrainBot extends Robot {
    public boolean headingLock = false;
    public double backDropLock = Math.toRadians(90);
    public double collectLock = Math.toRadians(-90);
    ElapsedTime timer = new ElapsedTime();
    public int armPosition = 0;
    public boolean stateFinished;
    public long startOfSequencerTime;
    public int oldArmPosition = 0;
    public long waitTime;
    private HardwareMap hardwareMap;
    public DcMotor frontLeft, frontRight, backLeft, backRight, armMotor, chinUpMotor;
    public DcMotor odometerR, odometerL, odometerA;
    public IMU imu;
    public Servo liftServo;
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

    public XDrivetrainBot(String string) {
        this.engine = engine;
        setup();
        this.string = string;
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
        armMotor = engine.hardwareMap.dcMotor.get("arm");
        chinUpMotor = engine.hardwareMap.dcMotor.get("chinUpMotor");

        //SERVOS
        liftServo = engine.hardwareMap.servo.get("lift");


//        configuration = new TimeCraftersConfiguration("Blue Crab");


        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        chinUpMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chinUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

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
        }