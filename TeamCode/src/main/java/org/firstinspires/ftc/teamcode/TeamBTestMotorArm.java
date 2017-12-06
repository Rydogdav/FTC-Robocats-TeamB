
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

@TeleOp(name="Arm Test", group="Linear Opmode")
@Disabled

public class TeamBTestMotorArm extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    static final int armStartDegree = 45; //Relative to the ground perpendicularly
    static final int armEndDegree = 200; //Relative to the ground perpendicularly
    static final double COUNTS_PER_MOTOR60_REV = 1220;
    static final double COUNTS_PER_MOTOR40_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double SERVO_UP = 1.0;
    static final double SERVO_DOWN = 0.7;


    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor motorArm = null;
    public ColorSensor colorSensor = null;
    public Servo servoArm = null;
    public int bPressed = 1;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        //motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        servoArm = hardwareMap.get(Servo.class, "servoArm");

        //motorLeft.setDirection(DcMotor.Direction.FORWARD);
        //motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        telemetry.addLine("Running Arm Test!");
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
            /*if (gamepad1.x) {
                colorServo.setPosition(SERVO_DOWN);
            }*/
            telemetry.addData("bPressed = ", bPressed);
            telemetry.update();
            idle();
        }
    }
}
