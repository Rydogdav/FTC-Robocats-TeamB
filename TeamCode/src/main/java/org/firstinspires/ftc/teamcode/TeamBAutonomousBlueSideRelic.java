package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Autonomous BLUE RELIC Alpha 1.0", group="Autonomous")

public class TeamBAutonomousBlueSideRelic extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1220;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.5;
    public DcMotor motorFLeft = null;
    public DcMotor motorFRight = null;
    public ColorSensor colorSensor = null;
    public Servo colorServo = null;
    public boolean redTeam;
    public boolean blueTeam;
    public boolean confirmation = false;
    public String decision;
    //public ServoControllerEx servoControl = null;
    public final int SERVO_PORT = 2;


    @Override
    public void runOpMode() {

        motorFLeft = hardwareMap.get(DcMotor.class, "motorFLeft");
        motorFRight = hardwareMap.get(DcMotor.class, "motorFRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorServo = hardwareMap.get(Servo.class, "colorServo");

        motorFLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFRight.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        motorFLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d", motorFLeft.getCurrentPosition(), motorFRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //servoControl.setServoPwmEnable(SERVO_PORT);
        jewelDrive();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double time) {
        int motorFLeftTarget;
        int motorFRightTarget;

        if (opModeIsActive()) {

            motorFLeftTarget = motorFLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            motorFRightTarget = motorFRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            motorFLeft.setTargetPosition(motorFLeftTarget);
            motorFRight.setTargetPosition(motorFRightTarget);

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorFLeft.setPower(speed);
            motorFRight.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < time) && (motorFLeft.isBusy() && motorFRight.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d", motorFLeftTarget, motorFRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", motorFLeft.getCurrentPosition(), motorFRight.getCurrentPosition());
                telemetry.update();
            }

            motorFLeft.setPower(0);
            motorFRight.setPower(0);

            motorFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void jewelDrive() {
        colorServo.setPosition(0);
        sleep(1000);
        pushRedJewel(4);
    }

    public void pushRedJewel(double jewelInches) {
        //double jewelEquation = jewelInches / 2.54;
        if (colorSensor.red() > colorSensor.blue()) {
            encoderDrive(0.15, jewelInches, jewelInches, 1.5);
            parkOnCryptobox(30);
        }
        else if (colorSensor.blue() > colorSensor.red()) {
            encoderDrive(0.15, -jewelInches, -jewelInches, 1.5);
            parkOnCryptobox(34);
        }
        else {
            parkOnCryptobox(32);
        }
    }

    public void parkOnCryptobox(double distance) {
        colorServo.setPosition(1.0);
        sleep(1000);
        encoderDrive(0.15,  distance,  distance, 5.0);
    }
}

