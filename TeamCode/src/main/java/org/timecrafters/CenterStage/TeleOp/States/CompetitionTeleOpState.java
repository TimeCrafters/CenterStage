package org.timecrafters.CenterStage.TeleOp.States;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

@Config

public class CompetitionTeleOpState extends CyberarmState {
    // ------------------------------------------------------------------------------------------------------------- State and engine setup:
    private CompetitionRobotV1 robot;
    // ------------------------------------------------------------------------------------------------------------- Heading lock variables:
    public double integralSum = 0;
    private double targetHeading;
    public double collectLock = Math.toRadians(-90);
    public double backDropLock = Math.toRadians(90);

    public double power;
    private double currentHeading;
    private boolean headingLock = false;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // ---------------------------------------------------------------------------------------------------- collector / depositor variables:
    public static float leftOpen = 0;
    public static float leftClose = 0;
    public static float rightOpen = 0;
    public static float rightClose = 0;
    public float collectorPosLeft;
    public float collectorPosRight;
    public boolean lbsVar2;
    public boolean rbsVar2;
    public boolean bVar2;

    //---------------------------------------------------------------------------------------------------------------- Drivetrain Variables:
    private boolean lbsVar1;
    private double drivePower = 1;
    public double rx;

    // ------------------------------------------------------------------------------------------------------------------- Slider Variables:
    private int maxExtension = 2000;
    private int minExtension = 0;
    public boolean depositMode = false;

    // ---------------------------------------------------------------------------------------------------------------Arm control Variables:
    public String armPos = "collect";
    public static float shoulderCollect = 0;
    public static float shoulderDeposit = 0;
    public static float shoulderPassive = 0;
    public static float elbowCollect = 0;
    public static float elbowDeposit = 0;
    public static float elbowPassive = 0;



    public CompetitionTeleOpState(CompetitionRobotV1 robot) {
        this.robot = robot;

    }

    //    }
    // --------------------------------------------------------------------------------------------------------- Slider control function
    private void SliderTeleOp() {

        if (depositMode) {
            if (engine.gamepad2.right_trigger != 0) {
                if (robot.lift.getCurrentPosition() >= maxExtension) {
                    robot.lift.setPower(0);
                } else if (robot.lift.getCurrentPosition() >= maxExtension - 200) {
                    robot.lift.setPower(0.35);
                } else {
                    robot.lift.setPower(engine.gamepad2.right_trigger);
                }
            } else if (engine.gamepad2.left_trigger != 0) {

                if (robot.lift.getCurrentPosition() <= minExtension) {
                    robot.lift.setPower(0);
                } else if (robot.lift.getCurrentPosition() < 350) {
                    robot.lift.setPower(-0.3);
                } else {
                    robot.lift.setPower(-engine.gamepad2.left_trigger);
                }
            } else {
                robot.lift.setPower(0);
            }
        } else {
            robot.lift.setPower(0);
        }
    }


    public void DriveTrainTeleOp () {
        if (headingLock){
            robot.rx = HeadingPIDControl(targetHeading, currentHeading);
        } else {
            robot.rx = engine.gamepad1.right_stick_x;

        }
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
        double heading = -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // joystick math to find the approximate power across each wheel for a movement
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY + rotX - rx) / denominator;
        double backRightPower = (rotY - rotX - rx) / denominator;

        // setting each power determined previously from the math above
        // as well as multiplying it by a drive power that can be changed.
        robot.backLeft.setPower(backLeftPower * drivePower);
        robot.backRight.setPower(backRightPower * drivePower);
        robot.frontLeft.setPower(frontLeftPower * drivePower);
        robot.frontRight.setPower(frontRightPower * drivePower);
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public double HeadingPIDControl(double reference, double current){
        double error = angleWrap(current - reference);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public void ClawControlTeleOp() {
        boolean b2 = engine.gamepad2.b;
        if (b2 && !bVar2) {
            if (collectorPosLeft == leftClose && collectorPosRight == rightClose) {
                collectorPosLeft = leftOpen;
                collectorPosRight = rightOpen;
            } else {
                collectorPosLeft = leftClose;
                collectorPosRight = rightClose;
            }
        }
        bVar2 = b2;

        boolean lbs2 = engine.gamepad2.left_stick_button;
        if (lbs2 && !lbsVar2) {
            if (collectorPosLeft == leftClose) {
                collectorPosLeft = leftOpen;
            } else {
                collectorPosLeft = leftClose;
            }
        }
        lbsVar2 = lbs2;

        boolean rbs2 = engine.gamepad2.right_stick_button;
        if (rbs2 && !rbsVar2) {
            if (collectorPosRight == rightClose) {
                collectorPosRight = rightOpen;
            } else {
                collectorPosRight = rightClose;
            }
        }
        rbsVar2 = rbs2;
    }

    public void ArmPosControl(){

        if (engine.gamepad2.a){
            armPos = "collect";
            depositMode = false;
        } else if (engine.gamepad2.x){
            armPos = "passive";
            depositMode = false;
        } else if (engine.gamepad2.y){
            armPos = "deposit";
            depositMode = true;
        }

        if (armPos == "collect"){
            if (robot.lift.getCurrentPosition() >= 1){
                robot.lift.setPower(-0.3);
            } else {
                robot.lift.setPower(0);
                robot.shoulder.setPosition(shoulderCollect);
                robot.elbow.setPosition(elbowCollect);
            }
        }
        if (armPos == "passive"){
            if (robot.lift.getCurrentPosition() >= 1){
                robot.lift.setPower(-0.3);
            } else {
                robot.lift.setPower(0);
                robot.shoulder.setPosition(shoulderPassive);
                robot.elbow.setPosition(elbowPassive);
            }
        }
        if (armPos == "deposit"){
                robot.shoulder.setPosition(shoulderDeposit);
                robot.elbow.setPosition(elbowDeposit);
            }
        }

    @Override
    public void init() {
    }

    @Override
    public void exec() {
        // ---------------------------------------------------------------------------------------------------------- Game Pad 1, drivetrain

        currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (engine.gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        if (headingLock){
            robot.rx = HeadingPIDControl(targetHeading, currentHeading);
        } else {
            robot.rx = engine.gamepad1.right_stick_x;

        }

        // drivetrain
        DriveTrainTeleOp();

        if (engine.gamepad1.b){
            headingLock = true;
            targetHeading = backDropLock;
        }
        if (engine.gamepad1.x){
            headingLock = true;
            targetHeading = collectLock;
        }
        if (engine.gamepad1.a){
            headingLock = true;
            targetHeading = currentHeading;
        }
        if (engine.gamepad1.right_stick_x != 0){
            headingLock = false;
        }

        // ---------------------------------------------------------------------------------------- Game Pad 2, arms, claw, drone, and lift:

        // ------------------------------------------------------------------------------------------------------------------- Lift Control:
        SliderTeleOp();
        ClawControlTeleOp();
        ArmPosControl();

    }

        @Override
        public void telemetry () {
            engine.telemetry.addData("Lift Encoder Pos", robot.lift.getCurrentPosition());
            engine.telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            engine.telemetry.addData("arm Pos", robot.armPosition);
            engine.telemetry.addData("old arm pos", robot.oldArmPosition);
            engine.telemetry.addData("pid power", power);
            engine.telemetry.addData("heading Lock?", headingLock);
            engine.telemetry.addData("Kp", Kp);
            engine.telemetry.addData("Ki", Ki);
            engine.telemetry.addData("Kd", Kd);
        }
    }

