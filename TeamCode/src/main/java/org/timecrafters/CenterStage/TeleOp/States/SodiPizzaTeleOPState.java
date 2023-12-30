package org.timecrafters.CenterStage.TeleOp.States;

import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_COLLECT;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_DELIVER;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_PRECOLLECT;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_STOW;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.GRIPPER_CLOSED;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.GRIPPER_OPEN;

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
    private char buttonPressed;
    YawPitchRollAngles imuInitAngle;


    public SodiPizzaTeleOPState() {
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Arm should be at Position ", armPos);
        engine.telemetry.addData("Arm servo is at ", robot.shoulder.getPosition());

        engine.telemetry.addData("Button Pressed = ", buttonPressed);
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

        } else if (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < lastToldAngle - 0.5 &&
                Math.abs(engine.gamepad1.left_stick_y) > minInput) {

            robot.leftFront.setPower(robot.rightFront.getPower() * 0.8);
            robot.leftBack.setPower(robot.rightBack.getPower() * 0.8);

        } else {

            robot.leftFront.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        }

        if (engine.gamepad1.start && !engine.gamepad1.a) /*<-- reset everything: encoders, imu, and armPos int*/ {

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.imu.resetYaw();
        }

        if (Math.abs(engine.gamepad1.left_stick_y) > minInput &&
            Math.abs(engine.gamepad1.left_stick_x) < minInput &&
            armPos == 0/* &&
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > lastToldAngle - 0.5 &&
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < lastToldAngle + 0.5*/) {

            drivePower = engine.gamepad1.left_stick_y;
            robot.leftFront.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        }

        if (Math.abs(engine.gamepad1.left_stick_x) > minInput &&
            armPos == 0) {

            drivePower = engine.gamepad1.left_stick_x;
            robot.leftFront.setPower(-drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightBack.setPower(-drivePower);
        }

        if (Math.abs(engine.gamepad1.right_stick_x) > minInput &&
                armPos == 0) {

            drivePower = engine.gamepad1.right_stick_x;
            robot.leftFront.setPower(-drivePower);
            robot.rightFront.setPower(drivePower);
            robot.leftBack.setPower(-drivePower);
            robot.rightBack.setPower(drivePower);
            lastToldAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        // This moves the arm to Collect position, which is at servo position 0.00.
        if (engine.gamepad2.a && !engine.gamepad2.start) {
            armPos = 1;
        }

        if (armPos == 1) {

            buttonPressed = 'a';

            if (Math.abs(drivePower) > 0.25) {
                drivePower = 0.15;
                robot.leftFront.setPower(drivePower);
                robot.leftBack.setPower(drivePower);
                robot.rightFront.setPower(drivePower);
                robot.rightBack.setPower(drivePower);
            } else {
                //if servo's position is greater than Collect position with a run-to tolerance of 0.05,
                //decrement position at a rate of 0.05 per 150 milliseconds.
                if (robot.shoulder.getPosition() > ARM_PRECOLLECT && System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                    lastMoveTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(ARM_COLLECT);
                    armPos = 0;
                }

            }

        }
        //End of code for armPos = 1

        // This moves the arm to Precollect position, which is at servo position 0.05.
        if (engine.gamepad2.x) {
            armPos = 2;
        }

        if (armPos == 2) {

            buttonPressed = 'x';

            if (Math.abs(drivePower) > 0.25) {
                drivePower = 0.15;
                robot.leftFront.setPower(drivePower);
                robot.leftBack.setPower(drivePower);
                robot.rightFront.setPower(drivePower);
                robot.rightBack.setPower(drivePower);
            } else {
                //if servo's position is greater than Precollect position with a run-to tolerance of 0.05,
                //decrement position at a rate of 0.05 per 150 milliseconds.
                if (robot.shoulder.getPosition() > ARM_PRECOLLECT + 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                    lastMoveTime = System.currentTimeMillis();
                }//Incrementing from Collect position is unnecessary, because Collect is within the tolerance of run-to.
                 else if (System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(ARM_PRECOLLECT);
                    armPos = 0;
                }

            }
        }
        //End of code for armPos = 2

        // This moves the arm to Deliver position, which is at servo position 0.28.
        if (engine.gamepad2.b && !engine.gamepad2.start) {
            armPos = 3;
        }

        if (armPos == 3) {

            buttonPressed = 'b';

            if (Math.abs(drivePower) > 0.25) {
                drivePower = 0.15;
                robot.leftFront.setPower(drivePower);
                robot.leftBack.setPower(drivePower);
                robot.rightFront.setPower(drivePower);
                robot.rightBack.setPower(drivePower);
            } else {
                //if servo's position is less than Deliver position with a run-to tolerance of 0.05,
                //increment position at a rate of 0.05 per 150 milliseconds.
                if (robot.shoulder.getPosition() < ARM_DELIVER - 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() + 0.05);
                    lastMoveTime = System.currentTimeMillis();
                }//if servo's position is greater than Deliver position with a run-to tolerance of 0.05,
                //decrement position at a rate of 0.05 per 150 milliseconds.
                else if (robot.shoulder.getPosition() > ARM_DELIVER + 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                    lastMoveTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(ARM_DELIVER);
                    armPos = 0;
                }

            }
        }
        //End of code for armPos = 3

        // This moves the arm to Stow position, which is at servo position 0.72.
         if (engine.gamepad2.y) {
            armPos = 4;
        }

        if (armPos == 4) {

            buttonPressed = 'y';

            if (Math.abs(drivePower) > 0.25) {
                drivePower = 0.15;
                robot.leftFront.setPower(drivePower);
                robot.leftBack.setPower(drivePower);
                robot.rightFront.setPower(drivePower);
                robot.rightBack.setPower(drivePower);
            } else {
                //if servo's position is less than Collect position with a run-to tolerance of 0.05,
                //increment position at a rate of 0.05 per 150 milliseconds.
                if (robot.shoulder.getPosition() < ARM_STOW - 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(robot.shoulder.getPosition() + 0.05);
                    lastMoveTime = System.currentTimeMillis();
                }
                //Decrementing is unnecessary, because servo is mechanically inhibited from further advancing.
                else if (System.currentTimeMillis() - lastMoveTime >= 150) {
                    robot.shoulder.setPosition(ARM_STOW);
                    armPos = 0;
                }

            }
        }
        //End of code for armPos = 4

        if (engine.gamepad2.dpad_left) {
            robot.gripper.setPosition(GRIPPER_OPEN);
        } else if (engine.gamepad2.dpad_right) {
            robot.gripper.setPosition(GRIPPER_CLOSED);
        }



        }

    }
