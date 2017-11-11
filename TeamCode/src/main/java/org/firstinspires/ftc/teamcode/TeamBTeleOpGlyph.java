package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.sql.Time;

@TeleOp(name="Tele Op Glyph Alpha 1.1", group="Linear Opmode")
public class TeamBTeleOpGlyph extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorLeft = null; //set motors to nothing
    public DcMotor motorRight = null; //set motors to nothing
    public int bPressed = 1;
    public DcMotor motorArm = null;
    public Servo servoArm = null;
    //public Servo colorServo = null;
    static final double SERVO_UP = 1.0;
    static final double SERVO_DOWN = 0.6;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        //colorServo = hardwareMap.get(Servo.class, "colorServo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // sets direction to the left motor
        motorRight.setDirection(DcMotor.Direction.REVERSE); // sets direction to the left motor
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            /*
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            // Send calculated power to wheels
            if (leftPower >= 0.1 || rightPower >= 0.1 || leftPower <= -0.1 || rightPower <= 0.1) {
                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);
                idle();
            }
            else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
            if (gamepad1.right_trigger >= 0.1) { //If the right trigger is pressed, set the position to the maximum position and the power to the trigger value, going forwards
                motorArm.setPower(gamepad1.right_trigger * 0.3);
            }
            else if (gamepad1.left_trigger >= 0.1) {
                motorArm.setPower(-gamepad1.left_trigger * 0.3);
            }
            else {
                motorArm.setPower(0);
            }
            if (gamepad1.b) {
                bPressed *= -1;
                sleep(250);
            }
            if (bPressed == 1) {
                servoArm.setPosition(SERVO_UP);
            }
            if (bPressed == -1) {
                servoArm.setPosition(SERVO_DOWN);
            }
            idle();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("bPressed = ", bPressed);
            telemetry.update();
        }
    }
}
