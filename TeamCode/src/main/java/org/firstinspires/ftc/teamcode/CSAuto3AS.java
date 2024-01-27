/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is an example LinearOpMode that shows how to use a color sensor in a generic
 * way, regardless of which particular make or model of color sensor is used. The Op Mode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 * There will be some variation in the values measured depending on the specific sensor you are using.
 *
 * You can increase the gain (a multiplier to make the sensor report higher values) by holding down
 * the A button on the gamepad, and decrease the gain by holding down the B button on the gamepad.
 *
 * If the color sensor has a light which is controllable from software, you can use the X button on
 * the gamepad to toggle the light on and off. The REV sensors don't support this, but instead have
 * a physical switch on them to turn the light on and off, beginning with REV Color Sensor V2.
 *
 * If the color sensor also supports short-range distance measurements (usually via an infrared
 * proximity sensor), the reported distance will be written to telemetry. As of September 2020,
 * the only color sensors that support this are the ones from REV Robotics. These infrared proximity
 * sensor measurements are only useful at very small distances, and are sensitive to ambient light
 * and surface reflectivity. You should use a different sensor if you need precise distance measurements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this Op Mode to the Driver Station OpMode list
 */
@Autonomous(name = "Blue")

public class CSAuto3AS extends LinearOpMode {

public void encoderDrive(double rspeed, double fspeed,
                             double backleftInches, double backrightInches,
                             double frontleftInches, double frontrightInches, double timeoutS) {
                                 
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newbackLeftTarget = backleftDrive.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newbackRightTarget = backrightDrive.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
            newfrontLeftTarget = frontleftDrive.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frontrightDrive.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            
            backleftDrive.setTargetPosition(newbackLeftTarget);
            backrightDrive.setTargetPosition(newbackRightTarget);
            frontleftDrive.setTargetPosition(newfrontLeftTarget);
            frontrightDrive.setTargetPosition(newfrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
            
            // reset the timeout time and start motion.
            runtime.reset();
            backleftDrive.setPower(Math.abs(rspeed));
            backrightDrive.setPower(Math.abs(rspeed));
            frontleftDrive.setPower(Math.abs(fspeed));
            frontrightDrive.setPower(Math.abs(fspeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive()  &&                 
            (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
               /* telemetry.addData("Running to",  " %7d :%7d  :%7d  :%7d", newfrontLeftTarget,  newfrontRightTarget,  
                                                                          newbackLeftTarget,  newbackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d  :%7d  :%7d",
                                            frontleftDrive.getCurrentPosition(), frontrightDrive.getCurrentPosition(),
                                            backleftDrive.getCurrentPosition(),  backrightDrive.getCurrentPosition());
                telemetry.update(); */
            }

            // Stop all motion;
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(10);   // optional pause after each move.
        }
    }
    
        public void slideEncoderDrive(double sSpeed, double slideRightInches,
                                    double slideLeftInches, double timeoutS) {
        
        int newSlideRightTarget;
        int newSlideLeftTarget;
        
        if (opModeIsActive()) {

            

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS)) {
                // Display it for the driver.
               /* telemetry.addData("Running to",  " %7d :%7d  :%7d  :%7d", newfrontLeftTarget,  newfrontRightTarget,  
                                                                          newbackLeftTarget,  newbackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d  :%7d  :%7d",
                                            frontleftDrive.getCurrentPosition(), frontrightDrive.getCurrentPosition(),
                                            backleftDrive.getCurrentPosition(),  backrightDrive.getCurrentPosition());
                telemetry.update(); */
            }

            // Stop all motion;

            sleep(0);   // optional pause after each move.
        }
                                    }
  /** The colorSensor field will contain a reference to our color sensor hardware object */
  NormalizedColorSensor colorSensor;

  /** The relativeLayout field is used to aid in providing interesting visual feedback
   * in this sample application; you probably *don't* need this when you use a color sensor on your
   * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
  View relativeLayout;

  /**
   * The runOpMode() method is the root of this Op Mode, as it is in all LinearOpModes.
   * Our implementation here, though is a bit unusual: we've decided to put all the actual work
   * in the runSample() method rather than directly in runOpMode() itself. The reason we do that is
   * that in this sample we're changing the background color of the robot controller screen as the
   * Op Mode runs, and we want to be able to *guarantee* that we restore it to something reasonable
   * and palatable when the Op Mode ends. The simplest way to do that is to use a try...finally
   * block around the main, core logic, and an easy way to make that all clear was to separate
   * the former from the latter in separate methods.
   */
    private DcMotor         backleftDrive   = null;
    private DcMotor         backrightDrive  = null;
    private DcMotor         frontleftDrive = null;
    private DcMotor         frontrightDrive = null;
    private DcMotor         arm = null;
    private Servo pin;
    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     SLIDE_DIAMETER_INCHES   = 1.43 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     SLIDE_COUNTS_PER_INCH   =(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (SLIDE_DIAMETER_INCHES * 3.14159);
    static final double     F_DRIVE_SPEED             = 1;
    static final double     R_DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.2;
    static final double     SLIDE_SPEED            =0.15;
    
  @Override public void runOpMode() {
    
        backleftDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        backrightDrive = hardwareMap.get(DcMotor.class, "rightBack");
        frontleftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontrightDrive = hardwareMap.get(DcMotor.class, "rightFront");
	arm = hardwareMap.get(DcMotor.class, "arm");
	pin = hardwareMap.get(Servo.class, "Intake");
        //redA = hardwareMap.get(DigitalChannel.class, "red");
        //greenA = hardwareMap.get(DigitalChannel.class, "green");
        //redB = hardwareMap.get(DigitalChannel.class, "red2");
        //greenB = hardwareMap.get(DigitalChannel.class, "green2");
        
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);//MAY NEED TO CHANGE TO REVERSE
        
        
        

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
       /* telemetry.addData("Starting at",  "%7d :%7d  :%7d  :%7d :%7d :%7d",
                          frontleftDrive.getCurrentPosition(),
                          frontrightDrive.getCurrentPosition(),
                          backleftDrive.getCurrentPosition(),
                          backrightDrive.getCurrentPosition());*/

    // Get a reference to the RelativeLayout so we can later change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
       
    

    // Wait for the start button to be pressed.
    waitForStart();

    // Loop until we are asked to stop
    while (opModeIsActive()) {
                pin.setPosition(0.4);
		arm.setPower(0.05);
                encoderDrive(F_DRIVE_SPEED/1.5, R_DRIVE_SPEED/1.5, -5.5, 5.5, 5.5, -5.5, 1.5);
                sleep(500);
                encoderDrive(F_DRIVE_SPEED, R_DRIVE_SPEED, 46, 46, 46, 46, 3.0);
		arm.setPower(0);
		pin.setPosition(1);
                encoderDrive(F_DRIVE_SPEED, R_DRIVE_SPEED, -6, -6, -6, -6, 3.0);
		arm.setPower(0.05);
		encoderDrive(F_DRIVE_SPEED/1.5, R_DRIVE_SPEED/1.5, -8, 8, 8, -8, 1.5);
		encoderDrive(F_DRIVE_SPEED, R_DRIVE_SPEED, 6, 6, 6, 6, 2.0);
                sleep(10000000);
                }
      // Explain basic gain information via telemetry
      
      /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
       * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
       * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
       * for an explanation of HSV color. */

      // Update the hsvValues array by passing it to Color.colorToHSV()

      // Change the Robot Controller's background color to match the color detected by the color sensor.
        }
    }

