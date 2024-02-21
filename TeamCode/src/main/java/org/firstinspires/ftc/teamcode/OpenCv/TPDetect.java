package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class TPDetect extends OpMode {

    OpenCvCamera webcam1 = null;
    @Override
    public void init(){

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "Id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new Blue_Pipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


    }

    class
    Blue_Pipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat RightCrop;
        Mat MidCrop;
        Mat LeftCrop;
        double RAvgFin;
        double LAvgFin;
        double MAvgFin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline running");

            Rect LeftRec = new Rect(1, 1, 425, 719);
            Rect MidRec = new Rect(426, 1, 425, 719);
            Rect RightRec = new Rect(720, 1, 425, 719);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, LeftRec, rectColor, 2);
            Imgproc.rectangle(outPut, RightRec, rectColor, 2);
            Imgproc.rectangle(outPut, MidRec, rectColor, 2);

            RightCrop = YCbCr.submat(RightRec);
            LeftCrop = YCbCr.submat(LeftRec);
            MidCrop = YCbCr.submat(MidRec);

            Core.extractChannel(LeftCrop, LeftCrop, 2);
            Core.extractChannel(RightCrop, RightCrop, 2);
            Core.extractChannel(MidCrop, MidCrop, 2);

            Scalar LAvg = Core.mean(LeftCrop);
            Scalar RAvg = Core.mean(RightCrop);
            Scalar MAvg = Core.mean(MidCrop);

            LAvgFin = LAvg.val[0];
            RAvgFin = RAvg.val[0];
            MAvgFin = MAvg.val[0];

            if (LAvgFin > MAvgFin) {
                    telemetry.addLine("Left");
            } else if (RAvgFin > MAvgFin) {
                    telemetry.addLine("Right");
            } else {
                    telemetry.addLine("Middle");
            }

            return(outPut);
        }
    }
    @Override
    public void loop(){

    }
}
