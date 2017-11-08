
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

public class TeamBTestMotorArm extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    static final int armStartDegree = 45; //Relative to the ground perpendicularly
    static final int armEndDegree = 200; //Relative to the ground perpendicularly
    static final double COUNTS_PER_MOTOR60_REV = 1220;
    static final double COUNTS_PER_MOTOR40_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;

    static final int motorArmEncoderMax = (int) Math.round(armEndDegree - armStartDegree / 360 * COUNTS_PER_MOTOR40_REV);


    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor motorArm = null;
    public ColorSensor colorSensor = null;
    public Servo colorServo = null;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorServo = hardwareMap.get(Servo.class, "colorServo");

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();
        waitForStart();
        runtime.reset();
        telemetry.addLine("Running Arm Test!");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_trigger >= 0.1){ //If the right trigger is pressed, set the position to the maximum position and the power to the trigger value, going forwards
                motorArm.setTargetPosition(motorArmEncoderMax);
                motorArm.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger >= 0.1) { //If the left trigger is pressed, set the maximum position to the home position and the power to the trigger value, going backwards
                motorArm.setTargetPosition(0);
                motorArm.setPower(-gamepad1.left_trigger);
            }
        }
    }
}
