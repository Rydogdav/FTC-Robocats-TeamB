package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.sql.Time;

@TeleOp(name="Tele Op Glyph Alpha 1.2", group="Linear Opmode")
public class TeamBTeleOpGlyph extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorFLeft = null; //set motors to nothing
    public DcMotor motorFRight = null; //set motors to nothing
    public DcMotor motorBLeft = null; //set motors to nothing
    public DcMotor motorBRight = null; //set motors to nothing
    public int rbPressed = 1;
    public int bPressed = 1;
    public int lbPressed = 1;
    public int xPressed = 1;
    public DcMotor motorArm = null;
    public Servo servoArm = null;
    public Servo servoArm2 = null;
    //public ServoControllerEx servoControl = null;
    public final int SERVO_PORT = 2;
    static final double SERVO_UP = 0.80; // ** OPEN **
    static final double SERVO_DOWN = 0.50; // ** CLOSED **
    static final double SERVO_HALF = 0.65; // ** HALF **
    public Servo colorServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFLeft = hardwareMap.get(DcMotor.class, "motorFLeft");
        motorFRight = hardwareMap.get(DcMotor.class, "motorFRight");
        motorBLeft = hardwareMap.get(DcMotor.class, "motorBLeft");
        motorBRight = hardwareMap.get(DcMotor.class, "motorBRight");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoArm2 = hardwareMap.get(Servo.class, "servoArm2");
        //servoControl = hardwareMap.get(ServoControllerEx.class, "Servo_Controller_1");
        colorServo = hardwareMap.get(Servo.class, "colorServo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFLeft.setDirection(DcMotor.Direction.FORWARD); // sets direction to the left motor
        motorFRight.setDirection(DcMotor.Direction.REVERSE); // sets direction to the left motor
        motorBLeft.setDirection(DcMotor.Direction.FORWARD); // sets direction to the left motor
        motorBRight.setDirection(DcMotor.Direction.REVERSE); // sets direction to the left motor
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double leftPower;
        double rightPower;
        double speed;
        double gripperState = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //servoControl.setServoPwmDisable(SERVO_PORT);
            colorServo.setPosition(1.0);

            // *****Drive code*****
            /*int slowMode;                                   // test code for left bumper slow mode
            if (gamepad1.left_bumper) slowMode = 1;
            else slowMode = 0;*/

            leftPower = gamepad1.left_stick_y; // removed "- (slowMode * gamepad1.left_stick_y * 0.7)"
            rightPower = gamepad1.right_stick_y; // removed "- (slowMode * gamepad1.right_stick_y * 0.7)"

//            if (gamepad1.left_bumper) {             // slow mode code
//                lbPressed *= -1;
//                sleep(250);
//            }

            if (gamepad1.left_bumper) {             // slow mode code
                speed = 0.5;
            }
            else {
                speed = 1.0;
            }

            //if (lbPressed == 1) {
                motorFLeft.setPower(leftPower * speed);
                motorFRight.setPower(rightPower * speed);
                motorBLeft.setPower(leftPower * speed);
                motorBRight.setPower(rightPower * speed);
            //}
            //if (lbPressed == -1) {
            //    motorFLeft.setPower(leftPower * 0.5);
            //    motorFRight.setPower(rightPower * 0.5);
            //    motorBLeft.setPower(leftPower * 0.5);
            //    motorBRight.setPower(rightPower * 0.5);
            //    idle();
            //}



            // *****Arm code*****
            if (gamepad1.right_trigger >= 0.1) {
                motorArm.setPower(gamepad1.right_trigger * 0.4); // move arm up
            }
            else if (gamepad1.left_trigger >= 0.1) {
                motorArm.setPower(-gamepad1.left_trigger * 0.2); // move arm down
            }
            else {
                motorArm.setPower(0.1);
            }

            // *****Gripper code*****
//            if (gamepad1.right_bumper) {
//                rbPressed *= -1;
//            if (gamepad1.b) {
//                bPressed *= -1;
//                sleep(250);
//            }
        if (gamepad1.right_bumper) {
            if (gripperState == 1)
                gripperState = -1;
            else
                gripperState = 1;
            sleep(250);
        }

        if (gamepad1.right_stick_button) {
                servoArm.setPosition(SERVO_HALF);
                servoArm2.setPosition(1.0 - SERVO_HALF);
        }
        else if (gripperState == 1){
            servoArm.setPosition(SERVO_UP);
            servoArm2.setPosition(1.0 - SERVO_UP);
        }
        else if (gripperState == -1){
            servoArm.setPosition(SERVO_DOWN);
            servoArm2.setPosition(1.0 - SERVO_DOWN);
        }

//            if (bPressed == -1) {                           // Gripper Ha
//                servoArm.setPosition(SERVO_HALF);
//                servoArm2.setPosition(1.0 - SERVO_HALF);
//            }
//            else if (bPressed == 1) {
//                if (rbPressed == 1) {
//                    servoArm.setPosition(SERVO_UP);
//                    servoArm2.setPosition(1.0 - SERVO_UP);
//                }
//                if (rbPressed == -1) {                      // Gripper closed
//                    servoArm.setPosition(SERVO_DOWN);
//                    servoArm2.setPosition(1.0 - SERVO_DOWN);
//                }
//            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("bPressed = ", rbPressed);
            telemetry.addLine("Right!!");
            telemetry.update();
            idle();
        }
    }
}
