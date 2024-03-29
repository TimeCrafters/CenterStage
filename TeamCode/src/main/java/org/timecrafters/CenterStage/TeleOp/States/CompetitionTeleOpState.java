package org.timecrafters.CenterStage.TeleOp.States;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import java.util.Objects;

import dev.cyberarm.engine.V2.CyberarmState;

@Config

public class CompetitionTeleOpState extends CyberarmState {
    // ------------------------------------------------------------------------------------------------------------- State and engine setup:
    private CompetitionRobotV1 robot;
    // ------------------------------------------------------------------------------------------------------------robot claw arm variables:
    private PIDController pidController;
    public  double p = 0.005, i = 0,  d = 0.0001, f = 0;
    public int target = 0;

    // ---------------------------------------------------------------------------------------------------------------------shoot variables:
    public static double releasePos = 0.45;
    public static double holdPos = 0.95                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    ;


    public double maxVelocityX;
    public double maxVelocityY;

    // ------------------------------------------------------------------------------------------------------------- Heading lock variables:
    public double integralSum = 0;
    private double targetHeading;
    public double collectLock = Math.toRadians(-90);
    public double backDropLock = Math.toRadians(90);
    public double boost;
    public double armPower;
    private double currentHeading;
    private boolean headingLock = false;

    public static double Kp = 0.8;
    public static double Ki = 0;
    public static double Kd = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // ---------------------------------------------------------------------------------------------------- collector / depositor variables:
    public static double leftOpen = 0.25;
    public static double leftClose = 0.6;
    public static double rightOpen = 0.6;
    public static double rightClose = 0.25;
    public double collectorPosLeft = leftClose;
    public double collectorPosRight = rightClose;
    public boolean lbsVar2;
    public boolean rbsVar2;
    public boolean bVar2;

    //---------------------------------------------------------------------------------------------------------------- Drivetrain Variables:
    private boolean lbsVar1;
    private double drivePower = 1;
    public double rx = engine.gamepad1.right_stick_x / 2;

    // ------------------------------------------------------------------------------------------------------------------- Slider Variables:
    private int maxExtension = 2000;
    private int minExtension = 0;
    public boolean depositMode = false;

    // ---------------------------------------------------------------------------------------------------------------Arm control Variables:
    public String armPos = "collect";
    // chin up servo
    public static double chinUpServoUp = 0.55;
    public static double chinUpServoDown = 0;
    public long lastExecutedTime;




    public CompetitionTeleOpState(CompetitionRobotV1 robot) {
        this.robot = robot;
        pidController = new PIDController(p, i, d);
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

        boolean lbs1 = engine.gamepad1.left_stick_button;
        if (lbs1 && !lbsVar1) {
            if (drivePower == 1) {
                drivePower = 0.35;
            } else {
                drivePower = 1;
            }
        }
        lbsVar1 = lbs1;


        double x = -(engine.gamepad1.left_stick_x);
        double y = (engine.gamepad1.left_stick_y);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // angle math to make things field oriented
        double heading = -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // joystick math to find the approximate power across each wheel for a movement
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // setting each power determined previously from the math above
        // as well as multiplying it by a drive power that can be changed.
        robot.backLeft.setPower(backLeftPower * drivePower);
        robot.backRight.setPower(-backRightPower * drivePower);
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
        boolean b2 = engine.gamepad1.y;
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

        boolean lbs2 = engine.gamepad1.left_bumper;
        if (lbs2 && !lbsVar2) {
            if (collectorPosLeft == leftClose) {
                collectorPosLeft = leftOpen;
            } else {
                collectorPosLeft = leftClose;
            }
        }
        lbsVar2 = lbs2;

        boolean rbs2 = engine.gamepad1.right_bumper;
        if (rbs2 && !rbsVar2) {
            if (collectorPosRight == rightClose) {
                collectorPosRight = rightOpen;
            } else {
                collectorPosRight = rightClose;
            }
        }
        rbsVar2 = rbs2;
    }

    public void ArmPosControl() {
        if (engine.gamepad2.a && !engine.gamepad2.back) {
            armPos = "collect";
            depositMode = false;
        } else if (engine.gamepad2.y && !engine.gamepad2.back) {
            armPos = "deposit";
            depositMode = true;
        } else if (engine.gamepad2.b && !engine.gamepad2.start) {
            armPos = "hover";
            depositMode = true;
        } else if (engine.gamepad2.x && !engine.gamepad2.start) {
            armPos = "passive";
            depositMode = true;
        } else if (engine.gamepad2.dpad_left && !engine.gamepad2.start) {
            armPos = "lift up";
            depositMode = true;
        } else if (engine.gamepad2.dpad_right && !engine.gamepad2.start) {
            armPos = "lift down";
            depositMode = false;
        } else if (engine.gamepad2.back) {
            armPos = "reset";
        } else if (engine.gamepad2.right_bumper) {
            armPos = "tuning up";
        } else if (engine.gamepad2.left_bumper) {
            armPos = "tuning down";
        }

        if (Objects.equals(armPos, "tuning up")) {
            if (robot.lift.getCurrentPosition() >= 20) {
                robot.chinUpServo.setPosition(chinUpServoDown);
                robot.lift.setPower(-0.6);
            } else {
                robot.lift.setPower(0);
                if (System.currentTimeMillis() - lastExecutedTime > 200){
                    robot.shoulderCollect = robot.shoulderCollect + 0.02;
                    lastExecutedTime = System.currentTimeMillis();
                }
                robot.shoulder.setPosition(robot.shoulderCollect);
                robot.elbow.setPosition(robot.elbowCollect);
                robot.chinUpServo.setPosition(chinUpServoDown);
                target = 0;
                armPos = "collect";

            }
        }
        if (Objects.equals(armPos, "tuning down")) {
            if (robot.lift.getCurrentPosition() >= 20) {
                robot.chinUpServo.setPosition(chinUpServoDown);
                robot.lift.setPower(-0.6);
            } else {
                robot.lift.setPower(0);
                if (System.currentTimeMillis() - lastExecutedTime > 200) {
                    robot.shoulderCollect = robot.shoulderCollect - 0.02;
                    lastExecutedTime = System.currentTimeMillis();
                }
                robot.shoulder.setPosition(robot.shoulderCollect);
                robot.elbow.setPosition(robot.elbowCollect);
                robot.chinUpServo.setPosition(chinUpServoDown);
                target = 0;
                armPos = "collect";

            }
        }
        if (Objects.equals(armPos, "collect")) {
            if (robot.lift.getCurrentPosition() >= 20) {
                robot.chinUpServo.setPosition(chinUpServoDown);
                robot.lift.setPower(-0.6);
            } else {
                robot.lift.setPower(0);
                robot.shoulder.setPosition(robot.shoulderCollect);
                robot.elbow.setPosition(robot.elbowCollect);
                robot.chinUpServo.setPosition(chinUpServoDown);
                target = 0;

            }
        }
        if (Objects.equals(armPos, "deposit")) {
            robot.shoulder.setPosition(robot.shoulderDeposit);
            robot.elbow.setPosition(robot.elbowDeposit);
            target = 400;
            robot.chinUpServo.setPosition(chinUpServoDown);


        }
        if (Objects.equals(armPos, "hover")) {
            if (robot.lift.getCurrentPosition() >= 20) {
                robot.chinUpServo.setPosition(chinUpServoDown);
                robot.lift.setPower(-0.6);
            } else {
                robot.shoulder.setPosition(robot.shoulderCollect);
                robot.elbow.setPosition(robot.elbowCollect);
                target = 120;
            }
        }

        if (Objects.equals(armPos, "lift up")) {
            robot.shoulder.setPosition(0.2);
            target = 850;
            robot.chinUpServo.setPosition(chinUpServoUp);
        }

        if (Objects.equals(armPos, "lift down")) {
            if (robot.lift.getCurrentPosition() >= 1) {
                robot.lift.setPower(-0.6);
                robot.chinUpServo.setPosition(chinUpServoDown);
            } else {
                robot.lift.setPower(0);
                robot.shoulder.setPosition(0.2);
                target = 850;
            }
        }

        if (armPos.equals("reset")) {
            robot.shoulder.setPosition(robot.shoulderPassive);
            target = -10000;
            if (robot.touchLeftArm.isPressed() || robot.touchRightArm.isPressed()) {
                robot.clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPos = "collect";
            }
        }
    }


    @Override
    public void init() {
            super.init();
            pidController = new PIDController(p, i, d);
            robot.imu.resetYaw();
        robot.clawArm.setTargetPosition(0);
        robot.clawArm.setPower(0);
        robot.clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lastExecutedTime = System.currentTimeMillis();


    }

    @Override
    public void exec() { //------------------------------------------------------------------------------------------------------ EXEC Start
        if (robot.xVelocity > maxVelocityX){
            maxVelocityX = robot.xVelocity;
        }
        if (robot.yVelocity > maxVelocityY){
            maxVelocityY = robot.yVelocity;
        }
        robot.OdometryLocalizer();

        if (engine.gamepad2.start && engine.gamepad2.x){
            robot.shootServo.setPosition(releasePos);
        } else {
            robot.shootServo.setPosition(holdPos);
        }


        if (engine.gamepad2.dpad_up) {
            robot.chinUp.setPower(-1);
        } else if (engine.gamepad2.dpad_down){
            robot.chinUp.setPower(1);
        } else {
            robot.chinUp.setPower(0);
        }
        // ---------------------------------------------------------------------------------------------------------- Game Pad 1, drivetrain
        if (engine.gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // drivetrain

        if (engine.gamepad1.b){
            headingLock = true;
            targetHeading = backDropLock;
        } else if (engine.gamepad1.x){
            headingLock = true;
            targetHeading = collectLock;
        } else if (engine.gamepad1.a){
            headingLock = true;
            targetHeading = currentHeading;
        } else if (engine.gamepad1.right_stick_x != 0){
            headingLock = false;
        }

        if (headingLock){
            rx = HeadingPIDControl(targetHeading, currentHeading);
        } else {
            rx = engine.gamepad1.right_stick_x / 2;
        }

        DriveTrainTeleOp();

        // ---------------------------------------------------------------------------------------- Game Pad 2, arms, claw, drone, and lift:
//        pidController.setPID(p, i, d);
//        int armCurrentPos = robot.clawArm.getCurrentPosition();
//        double pid = pidController.calculate(armCurrentPos, target);

        if (armPos.equals("reset")){
            robot.clawArm.setPower(armPower);
        } else {
            armPower = 0.4; // pid
        }
        robot.clawArm.setTargetPosition(target);
        robot.clawArm.setPower(armPower);

        // ------------------------------------------------------------------------------------------------------------------- Lift Control:
        SliderTeleOp();

        robot.leftClaw.setPosition(collectorPosLeft);
        robot.rightClaw.setPosition(collectorPosRight);
        ArmPosControl();

        ClawControlTeleOp();


        robot.velocityChecker();

    }

        @Override
        public void telemetry () {
            engine.telemetry.addData("x velocity max", maxVelocityX);
            engine.telemetry.addData("y velocity max", maxVelocityY);
            engine.telemetry.addData("Dnl1", robot.Dnl1);
            engine.telemetry.addData("Dnr2", robot.Dnr2);
            engine.telemetry.addData("x pos", robot.positionX);
            engine.telemetry.addData("y pos", robot.positionY);
            engine.telemetry.addData("h pos odo", Math.toDegrees(robot.positionH));
            engine.telemetry.addData("aux encoder", robot.currentAuxPosition);
            engine.telemetry.addData("left encoder", robot.currentLeftPosition);
            engine.telemetry.addData("right encoder", robot.currentRightPosition);
            engine.telemetry.addData("Lift Encoder Pos", robot.lift.getCurrentPosition());
            engine.telemetry.addData("imu", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            engine.telemetry.addData("imu", -robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            engine.telemetry.addData("imu", -robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
            engine.telemetry.addData("pid power", HeadingPIDControl(targetHeading, currentHeading));
            engine.telemetry.addData("rx power", rx);
            engine.telemetry.addData("heading Lock?", headingLock);
            engine.telemetry.addData("Kp", Kp);
            engine.telemetry.addData("Ki", Ki);
            engine.telemetry.addData("Kd", Kd);
            engine.telemetry.addData("arm pos", armPos);
            engine.telemetry.addData("arm pos ticks", robot.clawArm.getCurrentPosition());
            engine.telemetry.addData("left touch", robot.touchLeftArm.isPressed());
            engine.telemetry.addData("right touch", robot.touchRightArm.isPressed());
        }
    }

