package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.sql.Time;

@TeleOp(name="Tele Op Glyph Alpha 1.1", group="Linear Opmode")
public class TeamBTeleOpGlyph extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorLeft = null; //set motors to nothing
    public DcMotor motorRight = null; //set motors to nothing
    public DcMotor motorGlyph1 = null; //set motors to nothing
    public DcMotor motorGlyph2 = null; //set motors to nothing
    public DcMotor motorGlyph3 = null; //set motors to nothing
    public DcMotor motorGlyph4 = null; //set motors to nothing
    public int bPressed = 1;
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorGlyph1 = hardwareMap.get(DcMotor.class, "motorGlyph1");
        motorGlyph2 = hardwareMap.get(DcMotor.class, "motorGlyph2");
        motorGlyph3 = hardwareMap.get(DcMotor.class, "motorGlyph3");
        motorGlyph4 = hardwareMap.get(DcMotor.class, "motorGlyph4"); //LIFT SYSTEM!


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // sets direction to the left motor
        motorRight.setDirection(DcMotor.Direction.REVERSE); // sets direction to the left motor
        motorGlyph1.setDirection(DcMotor.Direction.FORWARD); // sets the first glyph motor direction.
        motorGlyph2.setDirection(DcMotor.Direction.FORWARD); // sets the second glyph motor direction.
        motorGlyph3.setDirection(DcMotor.Direction.FORWARD); // sets the third glyph motor direction.
        motorGlyph4.setDirection(DcMotor.Direction.FORWARD); // LIFT SYSTEM! sets the final glyph motor direction.

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
            }
            else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }

            if (gamepad1.b) {
                sleep(500);
                bPressed *= -1;
            }

            if (gamepad1.a && bPressed > 0) { //Makes a boolean statement for if b is pressed, Boolean is true or false
                motorGlyph1.setPower(-1.0);
                motorGlyph2.setPower(1.0);
            }
            else if (gamepad1.a && bPressed < 0) {
                motorGlyph1.setPower(1.0);
                motorGlyph2.setPower(-1.0);
            }
            else {
                motorGlyph1.setPower(0.0);
                motorGlyph2.setPower(0.0);
            }
            if (gamepad1.x && bPressed > 0) {
                motorGlyph3.setPower(-1.0);
                motorGlyph4.setPower(-1.0);
            }
            else if (gamepad1.x && bPressed < 0) {
                motorGlyph3.setPower(1.0);
                motorGlyph4.setPower(1.0);
            }
            else {
                motorGlyph3.setPower(0.0);
                motorGlyph4.setPower(0.0);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
