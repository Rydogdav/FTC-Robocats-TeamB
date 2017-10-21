package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC Robotics on 10/3/2017.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Autonomous Alpha 1.0", group="Autonomous")
@Disabled
public class TeamBAutonomous extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime(); 
    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public ColorSensor colorSensor = null;
    public Servo colorServo = null;
    public double servoDegrees;
    public double servoEquation = 1/255 * servoDegrees;
    public double jewelInches;
    public double jewelEquation = jewelInches / 2.54;
    public boolean redTeam;
    public boolean blueTeam;
    public boolean confirmation = false;
    public String decision;


    @Override
    public void runOpMode() {

        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (confirmation == false) { //Locks in the alliance that we are on
            telemetry.addData("Press B for Red Alliance\nPress X for Blue Alliance", decision);
            if (gamepad1.b) {
                decision = "Red";
                redTeam = true;
                confirmation = true;
            }
            if (gamepad1.x) {
                decision = "Blue";
                blueTeam = true;
                confirmation = true;
            }
        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /*encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout*/

        //jewelDrive(100, 10);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    //**ENCODER DRIVE**
    public void encoderDrive(double speed, double leftInches, double rightInches, double time) {
        int motorLeftTarget;
        int motorRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            motorLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            motorRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            motorLeft.setTargetPosition(motorLeftTarget);
            motorRight.setTargetPosition(motorRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(speed);
            motorRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < time) && (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", motorLeftTarget,  motorRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    } //**ENCODER DRIVE**

    //**JEWEL DRIVE**
    public void jewelDrive(double servoDegrees, double jewelInches) {  //Creates method named jewelDrive
        colorServo.setPosition(servoEquation); //Takes argument(servoDegrees) and inputs it into the servo equation(1/255 * ser)
        if (redTeam == true && colorSensor.red() <= 2) { //If we're on the red team & jewel = red, move back
            encoderDrive(DRIVE_SPEED,  -jewelEquation,  -jewelEquation, 1.0);
        }
        if (redTeam == true && colorSensor.blue() <= 2) { //If we're on the red team & jewel = blue, move forward
            encoderDrive(DRIVE_SPEED,  jewelEquation,  jewelEquation, 1.0);
        }
        if (blueTeam == true && colorSensor.red() <= 2) { //If we're on the blue team & jewel = red, move forward
            encoderDrive(DRIVE_SPEED,  jewelEquation,  jewelEquation, 1.0);
        }
        if (blueTeam == true && colorSensor.blue() <= 2) { //If we're on the blue team & jewel = blue, move back
            encoderDrive(DRIVE_SPEED,  -jewelEquation,  -jewelEquation, 1.0);
        }
    } //**JEWEL DRIVE**
}