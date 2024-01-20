package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

public class CameraVisionState extends CyberarmState {

    CompetitionRobotV1 robot;
    ExamplePipeline pipeline;
    private long initTime;
    private boolean finish = false;

    public CameraVisionState(CompetitionRobotV1 robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        super.init();


        int cameraMonitorViewId = engine.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", engine.hardwareMap.appContext.getPackageName());
        robot.webcam1 = OpenCvCameraFactory.getInstance().createWebcam(robot.webCamName, cameraMonitorViewId);
        robot.webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                robot.webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });

        pipeline = new ExamplePipeline();
        robot.webcam1.setPipeline(pipeline);

    }

    @Override
    public void start() {
        super.start();
        initTime = System.currentTimeMillis();

    }

    @Override
    public void exec() {
        robot.clawArmControl();
        if (System.currentTimeMillis() - initTime > 4000){
            robot.objectPos = pipeline.objectPos();
            setHasFinished(true);
        }

        // odometry driving ALWAYS
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();



    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("object pos", pipeline.objectPos());
        engine.telemetry.addData("left side pixel amount", pipeline.leftavgfin);
        engine.telemetry.addData("middle side pixel amount", pipeline.middleavgfin);
        engine.telemetry.addData("right side pixel amount", pipeline.rightavgfin);
        engine.telemetry.addData("object pos", pipeline.objectPos());


    }

    class ExamplePipeline extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat leftCrop;
        Mat middleCrop;
        Mat rightCrop;
        double leftavgfin;
        double middleavgfin;
        double rightavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            Rect leftRect = new Rect(1, 1, 212, 359);
            Rect rightRect = new Rect(213, 1, 212, 359);
            Rect middleRect = new Rect(426, 1, 212, 359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);
            Imgproc.rectangle(output, middleRect, rectColor, 2);

            leftCrop = HSV.submat(leftRect);
            rightCrop = HSV.submat(rightRect);
            middleCrop = HSV.submat(middleRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(middleCrop, middleCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar middleavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            middleavgfin = middleavg.val[0];

            HSV.release();
            leftCrop.release();
            middleCrop.release();
            rightCrop.release();

            return (output);


        }

        public int objectPos() {
            if (leftavgfin > rightavgfin && leftavgfin > middleavgfin) {
                return 1;
            }
            else if (rightavgfin > leftavgfin && rightavgfin > middleavgfin) {
                return 2;
            }
            else {
                return 3;
            }
        }
    }


}

