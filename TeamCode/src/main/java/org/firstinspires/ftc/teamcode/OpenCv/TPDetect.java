package org.firstinspires.ftc.teamcode.OpenCv;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TPDetect extends OpenCvPipeline {
    public static boolean Detect_Blue = true;
    public static double MinimumVal = 100;
    public static double MaximumVal = 255;
    public static double MinimumBHue = 100;
    public static double MaximumBHue = 170;
    public static double MinimumRLHue = 0;
    public static double MaximumRLHue = 30;
    public static double MinimumRHHue = 160;
    public static double MaximumRHHue = 255;

    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        Right,
        Left,
        Middle
    }
    private Location location;
    static final Rect LeftArea = new Rect(new Point(10,100), new Point(105,200));
    static final Rect RightArea = new Rect(new Point(10, 200), new Point(205, 200));
    static final Rect MiddleArea = new Rect(new Point(220,100), new Point(310,200));
    public CenterStageCVDetection(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MinB = new Scalar(MinimumBHue,MinimumVal,MinimumVal);
        Scalar MaxB = new Scalar(MaximumBHue,MaximumVal,MaximumVal);
    }
}

