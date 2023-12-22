package org.timecrafters.CenterStage.TeleOp.States;

import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_COLLECT;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_DELIVER;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_HOVER_5_STACK;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_PRECOLLECT;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.GamepadChecker;

public class SodiPizzaTeleOPState extends CyberarmState {

    final private SodiPizzaMinibotObject robot;
    private long lastMoveTime;
    public double drivePower;
    public final double minInput = 0.1 /* <- Minimum input from stick to send command */;
    public double lastToldAngle /* <- The angle the bot was last told to stop at */;
    private int armPos;
    YawPitchRollAngles imuInitAngle;


    public SodiPizzaTeleOPState() {
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Arm should be at Position ", armPos);
        engine.telemetry.addData("Arm servo is at ", robot.shoulder.getPosition());
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

        lastMoveTime = System.currentTimeMillis();
        armPos = 0;
    }

    @Override
    public void exec() {

        if (Math.abs(engine.gamepad1.left_stick_y) < minInput &&
                Math.abs(engine.gamepad1.left_stick_x) < minInput &&
                Math.abs(engine.gamepad1.right_stick_x) < minInput) /* <- input from ONLY left stick y means to move forward or backward */{

            drivePower = 0;
            robot.leftFront.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        }

        if (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > lastToldAngle + 0.5) {
            if (Math.abs(engine.gamepad1.right_stick_x) > minInput &&
            Math.abs(engine.gamepad1.left_stick_y) > minInput) {
                robot.rightFront.setPower(robot.leftFront.getPower() * 0.8);
                robot.rightBack.setPower(robot.leftBack.getPower() * 0.8);
            }

        } else
        if (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < lastToldAngle - 0.5 &&
                Math.abs(engine.gamepad1.left_stick_y) > minInput) {

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

        if (Math.abs(engine.gamepad1.left_stick_y) > minInput && Math.abs(engine.gamepad1.left_stick_x) < minInput/* &&
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > lastToldAngle - 0.5 &&
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < lastToldAngle + 0.5*/) {

            drivePower = engine.gamepad1.left_stick_y;
            robot.leftFront.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        }

        if (Math.abs(engine.gamepad1.left_stick_x) > minInput) {

            drivePower = engine.gamepad1.left_stick_x;
            robot.leftFront.setPower(-drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(-drivePower);
        }

        if (Math.abs(engine.gamepad1.right_stick_x) > minInput) {

            drivePower = engine.gamepad1.right_stick_x;
            robot.leftFront.setPower(-drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(-drivePower);
            robot.rightBack.setPower(drivePower);
            lastToldAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }


        if (engine.gamepad2.a && !engine.gamepad2.start) {
            armPos = 1;
        }

        if (armPos == 1) {

            if (Math.abs(drivePower) > 0.5) {
                drivePower = 0.15;
            } else {

                if (robot.shoulder.getPosition() > ARM_PRECOLLECT && System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                    lastMoveTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(ARM_COLLECT);
                    armPos = 0;
                }

            }

        }
        //End of code for armPos = 1


        if (engine.gamepad2.x) {
            armPos = 2;
        }

        if (armPos == 2) {

            if (Math.abs(drivePower) > 0.5) {
                drivePower = 0.15;
            } else {

                if (robot.shoulder.getPosition() > ARM_HOVER_5_STACK && System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                    lastMoveTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(ARM_PRECOLLECT);
                    armPos = 0;
                }

            }
        }
        //End of code for armPos = 2

        if (engine.gamepad2.b && !engine.gamepad2.start) {
            armPos = 3;
        }

        if (armPos == 3) {

            if (Math.abs(drivePower) > 0.5) {
                drivePower = 0.15;
            } else {

                if (robot.shoulder.getPosition() < ARM_DELIVER - 0.05 && System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                    lastMoveTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(ARM_DELIVER);
                    armPos = 0;
                }

            }
        }
        //End of code for armPos = 3


         if (engine.gamepad2.y) {
            armPos = 4;
        }

        if (armPos == 4) {

            if (Math.abs(drivePower) > 0.5) {
                drivePower = 0.15;
            } else {

                if (robot.shoulder.getPosition() > ARM_HOVER_5_STACK && System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                    lastMoveTime = System.currentTimeMillis();
                }
                else if (System.currentTimeMillis() - lastMoveTime >= 250) {
                    robot.shoulder.setPosition(ARM_PRECOLLECT);
                    armPos = 0;
                }

            }
        }
        //End of code for armPos = 4

        }

    }
