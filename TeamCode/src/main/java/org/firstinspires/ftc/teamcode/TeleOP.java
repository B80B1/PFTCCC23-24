package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;



@TeleOp(name = "Tele")

public class TeleOP extends LinearOpMode {

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;

    private DcMotor motor6 = null; //slide control left
    private Servo SJW;
    private DcMotor motor5 = null; //slide control right

    private Servo SC;
    private Servo PL;

    private Servo P;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotor.class, "leftFront"); //frontleft, port 0
        motor2 = hardwareMap.get(DcMotor.class, "rightFront");  //frontright, port 1
        motor3 = hardwareMap.get(DcMotor.class, "leftBack"); //backleft, port 3
        motor4 = hardwareMap.get(DcMotor.class, "rightBack");  //backright, port 2
        motor5 = hardwareMap.get(DcMotor.class, "arm");
        motor6 = hardwareMap.get(DcMotor.class, "arm t");
        SJW = hardwareMap.get(Servo.class, "wrist");
        PL = hardwareMap.get(Servo.class, "Plane J");
        P = hardwareMap.get(Servo.class, "Plane");




        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double forwardMotion;
            double horizonMotion;
            double rotateMotion;

            forwardMotion = gamepad1.right_stick_y;
            horizonMotion = gamepad1.right_stick_x;
            rotateMotion = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(forwardMotion) + Math.abs(horizonMotion) + Math.abs(rotateMotion), 1);
            double m1Power = (forwardMotion - horizonMotion - rotateMotion) / denominator;
            double m2Power = (forwardMotion + horizonMotion + rotateMotion) / denominator;
            double m3Power = (forwardMotion + horizonMotion - rotateMotion) / denominator;
            double m4Power = (forwardMotion - horizonMotion + rotateMotion) / denominator;
            double m5Power = gamepad2.right_stick_y;
            double m6Power = gamepad2.left_stick_y;

            motor1.setPower(m1Power);
            motor2.setPower(m2Power);
            motor3.setPower(m3Power);
            motor4.setPower(m4Power);
            motor5.setPower((m5Power + 0.01)/(2.0));
            motor6.setPower((-m6Power - 0.01)/2.0);

            if (gamepad2.a) {
                SJW.setPosition(0.3);
            } else if (gamepad2.b) {
                SJW.setPosition(0.7);
            } else {
                SJW.setPosition(0);
            }
            if (gamepad2.dpad_up) {
                PL.setPosition(96);
            } else if (gamepad2.dpad_down) {
                PL.setPosition(0);
            }
            if (gamepad2.dpad_right) {
                P.setPosition(.60);
            } else if (gamepad2.dpad_left) {
                P.setPosition(.30);
            } else {
                P.setPosition(.50);
            }



            telemetry.addData("m5", m5Power);
            //telemetry.addData("m6", motor6);
            telemetry.addData("FM", forwardMotion);
            telemetry.addData("HM", horizonMotion);
            telemetry.addData("RM", rotateMotion);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
