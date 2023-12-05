package org.timecrafters.CenterStage.TeleOp.States;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.Common.XDrivetrainBot;

import dev.cyberarm.engine.V2.CyberarmState;
@Config
public class XDrivetrainRobotState extends CyberarmState {
    private XDrivetrainBot robot;
    private PIDController ArmPidController;
    private float ticksInDegree = 144 / 180;

    public static double ArmP = 0, ArmI = 0, ArmD = 0, ArmF = 0;
    private double targetHeading;

    public double power;
    private double currentHeading;
    public double integralSum = 0;
    public double ArmintegralSum = 0;
    public static double liftServoPos = 0.5;
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double target;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double ArmlastError = 0;
    private boolean headingLock = false;

    public XDrivetrainRobotState(XDrivetrainBot robot) {
        this.robot = robot;

    }
    //    }
    // --------------------------------------------------------------------------------------------------------- Slider control function
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public double ArmControlPower(double reference, double current){
        double error = (current - reference);
        ArmintegralSum += error * timer.seconds();
        double derivative = (error - ArmlastError) / timer.seconds();

        timer.reset();

        double output = (error * ArmP) + (derivative * ArmD) + (integralSum * ArmI);
        return output;

    }

    public void chinUpControl(){
        if(engine.gamepad1.left_bumper){
            robot.chinUpMotor.setPower(-1);
        } else if(engine.gamepad1.right_bumper){
            robot.chinUpMotor.setPower(1);
        } else {
            robot.chinUpMotor.setPower(0);
        }

    }


    public double HeadingPIDControl(double reference, double current){
        double error = angleWrap(current - reference);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    @Override
    public void init() {
    }

    @Override
    public void exec() {

        if (engine.gamepad2.right_stick_button){
            robot.armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
        currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (engine.gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        if (headingLock){
            robot.rx = HeadingPIDControl(targetHeading, currentHeading);
        } else {
            robot.rx = engine.gamepad1.right_stick_x;

        }

        robot.armMotor.setPower(ArmControlPower(target, robot.armMotor.getCurrentPosition()));

        robot.liftServo.setPosition(liftServoPos);

            // drivetrain
            robot.driveTrainTeleOp();

            if (engine.gamepad1.b){
                headingLock = true;
                targetHeading = robot.backDropLock;
            }
            if (engine.gamepad1.x){
                headingLock = true;
                targetHeading = robot.collectLock;
            }
            if (engine.gamepad1.a){
                headingLock = true;
                targetHeading = currentHeading;
            }
            if (engine.gamepad1.right_stick_x != 0){
                headingLock = false;
            }

            chinUpControl();


    }

        @Override
        public void telemetry () {
            engine.telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            engine.telemetry.addData("arm Pos", robot.armPosition);
            engine.telemetry.addData("old arm pos", robot.oldArmPosition);
            engine.telemetry.addData("depositor pos", robot.depositorPos);
            engine.telemetry.addData("collector pos", robot.collectorPos);
            engine.telemetry.addData("time", System.currentTimeMillis() - robot.startOfSequencerTime);
            engine.telemetry.addData("heading lock?", headingLock);
            engine.telemetry.addData("pid heading power", HeadingPIDControl(targetHeading, currentHeading));
            engine.telemetry.addData("robot arm current pos ", robot.armMotor.getCurrentPosition());
            engine.telemetry.addData("arm pid power ", HeadingPIDControl(targetHeading, currentHeading));
            engine.telemetry.addData("p", ArmP);
            engine.telemetry.addData("i", ArmI);
            engine.telemetry.addData("d", ArmD);
            engine.telemetry.addData("f", ArmF);
        }
    }

