package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Autonomous RED Alpha 1.0", group="Autonomous")

public class TeamBAutonomousRed extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime(); 
    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.5;
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public ColorSensor colorSensor = null;
    public Servo colorServo = null;
    //public ServoControllerEx servoControl = null;
    public final int SERVO_PORT = 2;

    @Override
    public void runOpMode() {

        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorServo = hardwareMap.get(Servo.class, "colorServo");

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

        //servoControl.setServoPwmEnable(SERVO_PORT);
        jewelDrive();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

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
    public void jewelDrive() {  //Creates method named jewelDrive
        colorServo.setPosition(0.0);
        sleep(1000);
        pushBlueJewel(4);
    } //**JEWEL DRIVE**

    public void pushBlueJewel(double jewelInches) {
        if (colorSensor.red() > colorSensor.blue()) {
            encoderDrive(0.15,  jewelInches,  jewelInches, 1.0);
            //parkOnCryptobox(609);
        }
        else if (colorSensor.blue() > colorSensor.red()) {
            encoderDrive(0.15,  -jewelInches,  -jewelInches, 1.0);
            //parkOnCryptobox(609);
        }
        else {
            //parkOnCryptobox(609);
        }
    }
    public void parkOnCryptobox(double distance) {
        colorServo.setPosition(1.0);
        sleep(1000);
        encoderDrive(0.15,  distance,  distance, 5.0);
    }
}