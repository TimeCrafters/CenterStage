package org.timecrafters.CenterStage.TeleOp.States;

import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_COLLECT;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_DELIVER;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_PRECOLLECT;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.ARM_STOW;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.GRIPPER_CLOSED;
import static org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject.GRIPPER_OPEN;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.GamepadChecker;

public class SodiPizzaTeleOPState extends CyberarmState {

    final private SodiPizzaMinibotObject robot;
    private long lastMoveTime, lastDistRead /* <- last time Distance was read*/;
    public double drivePower;
    public final double minInput = 0.1 /* <- Minimum input from stick to send command */;
    public double lastToldAngle /* <- The angle the bot was last told to stop at */, proportion, integral, derivative;
    public float approxObjPos /* <- Approximate distance away the nearest obstruction is */, objData1, objData2, objData3;
    private double lfPower, rfPower, lbPower, rbPower;
    private float yTransitPercent, xTransitPercent, rotPercent, percentDenom;
    private int objectPos;
    private boolean droneLaunched;
    private char buttonPressed;
    private GamepadChecker gp1checker, gp2checker;
    YawPitchRollAngles imuInitAngle;


    public SodiPizzaTeleOPState() {
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    public float getApproxObjPos() {
        if (System.currentTimeMillis() - lastDistRead >= 500) {
            /*Pseudocode: take objData1, wait, take 2, wait, take 3*/

        }
        approxObjPos = (objData1 + objData2 + objData3)/3;
        return approxObjPos;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Launcher Servo: ", robot.launcher.getPosition());
        engine.telemetry.addData("Drone Launched?", droneLaunched);

        engine.telemetry.addLine();
        engine.telemetry.addData("Arm servo position", robot.shoulder.getPosition());
    }

    @Override
    public void init() {
        drivePower = 0;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        robot.launcher.setPosition(0);
        droneLaunched = false;

        robot.imu.resetYaw();
        imuInitAngle = robot.imu.getRobotYawPitchRollAngles();
        lastToldAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        gp1checker = new GamepadChecker(engine, engine.gamepad1);
        gp2checker = new GamepadChecker(engine, engine.gamepad2);

        lastMoveTime = System.currentTimeMillis();
        lastDistRead = System.currentTimeMillis();
    }

    @Override
    public void exec() {

        double y = -engine.gamepad1.left_stick_y; // Remember, this is reversed!
        double x = engine.gamepad1.left_stick_x;
        double rx = engine.gamepad1.right_stick_x;

        double lbPower = (y - x + rx);
        double rbPower = (y + x - rx);
        double lfPower = (y + x + rx);
        double rfPower = (y - x - rx);

        robot.leftFront.setPower(lfPower * drivePower);
        robot.leftBack.setPower(lbPower * drivePower);
        robot.rightFront.setPower(rfPower * drivePower);
        robot.rightBack.setPower(rbPower * drivePower);

        if (engine.gamepad1.left_stick_x > 0.1) {
            robot.leftBack.setPower(lbPower);
            robot.rightBack.setPower(rbPower);
            robot.leftFront.setPower(lfPower);
            robot.rightFront.setPower(rfPower);

        }


        if (engine.gamepad2.left_stick_button) {
            if (System.currentTimeMillis() - lastMoveTime >= 200) {
                robot.launcher.setPosition(0.98);
                lastMoveTime = System.currentTimeMillis();
            }
        } else if (engine.gamepad2.right_stick_button) {
            if (System.currentTimeMillis() - lastMoveTime >= 100) {
                robot.launcher.setPosition(robot.launcher.getPosition() - 0.2);
                lastMoveTime = System.currentTimeMillis();
            }
        } else if (robot.launcher.getPosition() >= 0.95) {
            if (System.currentTimeMillis() - lastMoveTime >= 1000) {
                droneLaunched = true;
                lastMoveTime = System.currentTimeMillis();
            }
        }

        if (!engine.gamepad2.left_stick_button && droneLaunched) {
            if (System.currentTimeMillis() - lastMoveTime >= 200) {
                robot.launcher.setPosition(robot.launcher.getPosition() - 0.025);
                lastMoveTime = System.currentTimeMillis();
            }
        }

        if (engine.gamepad2.left_stick_y > 0.1) {
            if (System.currentTimeMillis() - lastMoveTime >= 200) {
                robot.shoulder.setPosition(robot.shoulder.getPosition() + 0.05);
                lastMoveTime = System.currentTimeMillis();
            }
        } else if (engine.gamepad2.left_stick_y < -0.1) {
            if (System.currentTimeMillis() - lastMoveTime >= 200) {
                robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
                lastMoveTime = System.currentTimeMillis();
            }
        }

//        // This moves the arm to Collect position, which is at servo position 0.00.
//        if (engine.gamepad2.a && !engine.gamepad2.start) {
//            armPos = 1;
//        }
//
//        if (armPos == 1) {
//
//            buttonPressed = 'a';
//
//            if (Math.abs(drivePower) > 0.25) {
//                drivePower = 0.15;
//                robot.leftFront.setPower(drivePower);
//                robot.leftBack.setPower(drivePower);
//                robot.rightFront.setPower(drivePower);
//                robot.rightBack.setPower(drivePower);
//            } else {
//                //if servo's position is greater than Collect position with a run-to tolerance of 0.05,
//                //decrement position at a rate of 0.05 per 150 milliseconds.
//                if (robot.shoulder.getPosition() > ARM_PRECOLLECT && System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
//                    lastMoveTime = System.currentTimeMillis();
//                } else if (System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(ARM_COLLECT);
//                    armPos = 0;
//                }
//
//            }
//
//        }
//        //End of code for armPos = 1
//
//        // This moves the arm to Precollect position, which is at servo position 0.05.
//        if (engine.gamepad2.x) {
//            armPos = 2;
//        }
//
//        if (armPos == 2) {
//
//            buttonPressed = 'x';
//
//            if (Math.abs(drivePower) > 0.25) {
//                drivePower = 0.15;
//                robot.leftFront.setPower(drivePower);
//                robot.leftBack.setPower(drivePower);
//                robot.rightFront.setPower(drivePower);
//                robot.rightBack.setPower(drivePower);
//            } else {
//                //if servo's position is greater than Precollect position with a run-to tolerance of 0.05,
//                //decrement position at a rate of 0.05 per 150 milliseconds.
//                if (robot.shoulder.getPosition() > ARM_PRECOLLECT + 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
//                    lastMoveTime = System.currentTimeMillis();
//                }//Incrementing from Collect position is unnecessary, because Collect is within the tolerance of run-to.
//                 else if (System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(ARM_PRECOLLECT);
//                    armPos = 0;
//                }
//
//            }
//        }
//        //End of code for armPos = 2
//
//        // This moves the arm to Deliver position, which is at servo position 0.28.
//        if (engine.gamepad2.b && !engine.gamepad2.start) {
//            armPos = 3;
//        }
//
//        if (armPos == 3) {
//
//            buttonPressed = 'b';
//
//            if (Math.abs(drivePower) > 0.25) {
//                drivePower = 0.15;
//                robot.leftFront.setPower(drivePower);
//                robot.leftBack.setPower(drivePower);
//                robot.rightFront.setPower(drivePower);
//                robot.rightBack.setPower(drivePower);
//            } else {
//                //if servo's position is less than Deliver position with a run-to tolerance of 0.05,
//                //increment position at a rate of 0.05 per 150 milliseconds.
//                if (robot.shoulder.getPosition() < ARM_DELIVER - 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(robot.shoulder.getPosition() + 0.05);
//                    lastMoveTime = System.currentTimeMillis();
//                }//if servo's position is greater than Deliver position with a run-to tolerance of 0.05,
//                //decrement position at a rate of 0.05 per 150 milliseconds.
//                else if (robot.shoulder.getPosition() > ARM_DELIVER + 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.05);
//                    lastMoveTime = System.currentTimeMillis();
//                } else if (System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(ARM_DELIVER);
//                    armPos = 0;
//                }
//
//            }
//        }
//        //End of code for armPos = 3
//
//        // This moves the arm to Stow position, which is at servo position 0.72.
//         if (engine.gamepad2.y) {
//            armPos = 4;
//        }
//
//        if (armPos == 4) {
//
//            buttonPressed = 'y';
//
//            if (Math.abs(drivePower) > 0.25) {
//                drivePower = 0.15;
//                robot.leftFront.setPower(drivePower);
//                robot.leftBack.setPower(drivePower);
//                robot.rightFront.setPower(drivePower);
//                robot.rightBack.setPower(drivePower);
//            } else {
//                //if servo's position is less than Collect position with a run-to tolerance of 0.05,
//                //increment position at a rate of 0.05 per 150 milliseconds.
//                if (robot.shoulder.getPosition() < ARM_STOW - 0.05 && System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(robot.shoulder.getPosition() + 0.05);
//                    lastMoveTime = System.currentTimeMillis();
//                }
//                //Decrementing is unnecessary, because servo is mechanically inhibited from further advancing.
//                else if (System.currentTimeMillis() - lastMoveTime >= 150) {
//                    robot.shoulder.setPosition(ARM_STOW);
//                    armPos = 0;
//                }
//
//            }
//        }
        //End of code for armPos = 4

//        if (engine.gamepad2.dpad_left) {
//            robot.gripper.setPosition(GRIPPER_OPEN);
//        } else if (engine.gamepad2.dpad_right) {
//            robot.gripper.setPosition(GRIPPER_CLOSED);
//        }

        gp1checker.update();
        gp2checker.update();

        }

    }
