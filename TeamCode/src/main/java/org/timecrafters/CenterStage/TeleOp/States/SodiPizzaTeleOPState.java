package org.timecrafters.CenterStage.TeleOp.States;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.GamepadChecker;

public class SodiPizzaTeleOPState extends CyberarmState {

    final private SodiPizzaMinibotObject robot;
    private long lastMoveTime;
    public float drivePower;
    public double lastToldAngle /** <- The angle the bot was last told to stop at **/;
    YawPitchRollAngles imuInitAngle;


    public SodiPizzaTeleOPState() {
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    @Override
    public void init() {
        drivePower = 0;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        robot.imu.resetYaw();
        imuInitAngle = robot.imu.getRobotYawPitchRollAngles();
        lastToldAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        GamepadChecker gamepad1Checker = new GamepadChecker(engine, engine.gamepad1);
        GamepadChecker gamepad2Checker = new GamepadChecker(engine, engine.gamepad2);
    }

    @Override
    public void exec() {
        

        if (Math.abs(engine.gamepad1.left_stick_y) < 0.1 &&
                Math.abs(engine.gamepad1.left_stick_x) < 0.1 &&
                Math.abs(engine.gamepad1.right_stick_x) < 0.1) {

            drivePower = 0;
            robot.leftFront.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        }

        if (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > lastToldAngle + 0.5 &&
                Math.abs(engine.gamepad1.left_stick_y) > 0.1) {

            robot.rightFront.setPower(robot.leftFront.getPower() * 0.8);
            robot.rightBack.setPower(robot.leftBack.getPower() * 0.8);

        } else
        if (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < lastToldAngle - 0.5 &&
                Math.abs(engine.gamepad1.left_stick_y) > 0.1) {

            robot.leftFront.setPower(robot.rightFront.getPower() * 0.8);
            robot.leftBack.setPower(robot.rightBack.getPower() * 0.8);

        } else {

            robot.leftFront.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        }

        if (engine.gamepad1.start && !engine.gamepad1.a) {

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.imu.resetYaw();
        }

        if (Math.abs(engine.gamepad1.left_stick_y) > 0.1 && Math.abs(engine.gamepad1.left_stick_x) < 0.1 &&
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > lastToldAngle - 0.5 &&
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < lastToldAngle + 0.5) {

            drivePower = engine.gamepad1.left_stick_y;
            robot.leftFront.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        }

        if (Math.abs(engine.gamepad1.left_stick_x) > 0.1) {

            drivePower = engine.gamepad1.left_stick_x;
            robot.leftFront.setPower(-drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(-drivePower);
        }

        if (Math.abs(engine.gamepad1.right_stick_x) > 0.1) {

            drivePower = engine.gamepad1.right_stick_x;
            robot.leftFront.setPower(-drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(-drivePower);
            robot.rightBack.setPower(drivePower);
            lastToldAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }
}
