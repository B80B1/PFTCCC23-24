package org.firstinspires.ftc.teamcode.OpenCv.Detectors;

import static org.opencv.imgproc.Imgproc.COLOR_HSV2RGB;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TPDetectR extends OpenCvPipeline {
    public static boolean Detect_Red = true;
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
    public Location location;
    static final Rect LeftArea = new Rect(new Point(1,465), new Point(41,389));
    static final Rect MiddleArea = new Rect(new Point(622, 406), new Point(690, 365));
    static final Rect RightArea = new Rect(new Point(1230,570), new Point(1276,636));
    public TPDetectR(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MinB = new Scalar(MinimumBHue,MinimumVal,MinimumVal);
        Scalar MaxB = new Scalar(MaximumBHue,MaximumVal,MaximumVal);
        Scalar MinRL = new Scalar(MinimumRLHue,MinimumVal,MinimumVal);
        Scalar MaxRL = new Scalar(MaximumRLHue,MaximumVal,MaximumVal);
        Scalar MinRH = new Scalar(MinimumRHHue,MinimumVal,MinimumVal);
        Scalar MaxRH = new Scalar(MaximumRHHue,MaximumVal,MaximumVal);

        if (!Detect_Red) {
            Core.inRange(mat, MinB, MaxB, mat);
        } else {
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat, MinRL, MaxRL, mat);
            Core.inRange(mat, MinRH, MaxRH, mat);
            Core.bitwise_or(mat1, mat2, mat);
        }

        Mat Left = mat.submat(LeftArea);
        Mat Right = mat.submat(RightArea);
        Mat Middle = mat.submat(MiddleArea);

        double leftVal = Core.sumElems(Left).val[0];
        double rightVal = Core.sumElems(Right).val[0];
        double middleVal = Core.sumElems(Middle).val[0];

        telemetry.addData("Left Raw Value", leftVal);
        telemetry.addData("Right Raw Value", rightVal);
        telemetry.addData("Middle Raw Value", middleVal);

        Left.release();
        Right.release();
        Middle.release();

        if (leftVal > rightVal && leftVal > middleVal) {
                location = Location.Left;
                telemetry.addData("Prop at", "Right");
        } else if (rightVal > middleVal && rightVal > leftVal) {
                location = Location.Right;
                telemetry.addData("Prop at", "Left");
        } else if (middleVal >= rightVal && middleVal >= leftVal) {
                location = Location.Middle;
                telemetry.addData("Prop at", "Middle");
        }

        telemetry.update();

        Imgproc.cvtColor(mat, mat, COLOR_HSV2RGB);
        Scalar pixelColor = new Scalar(255, 255, 255);
        Scalar propColor = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LeftArea, location == Location.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, RightArea, location == Location.Right? pixelColor:propColor);
        Imgproc.rectangle(mat, MiddleArea, location == Location.Middle? pixelColor:propColor);

        return mat;
    }
}

