package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tele Op Glyph Alpha 1.0", group="Linear Opmode")
public class TeamBTeleOpGlyph extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor glyphMotor1 = null;
    public DcMotor glyphMotor2 = null;
    public DcMotor glyphMotor3 = null;
    public DcMotor glyphMotor4 = null;
    public boolean bPressed = false;
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        glyphMotor1 = hardwareMap.get(DcMotor.class, "glyphMotor1");
        glyphMotor2 = hardwareMap.get(DcMotor.class, "glyphMotor2");
        glyphMotor3 = hardwareMap.get(DcMotor.class, "glyphMotor3");
        glyphMotor4 = hardwareMap.get(DcMotor.class, "glyphMotor4");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        glyphMotor1.setDirection(DcMotor.Direction.FORWARD);
        glyphMotor2.setDirection(DcMotor.Direction.FORWARD);
        glyphMotor3.setDirection(DcMotor.Direction.FORWARD);
        glyphMotor4.setDirection(DcMotor.Direction.FORWARD);

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
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

                // Send calculated power to wheels
                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

            if (gamepad1.b) { //Makes a boolean statement for if b is pressed
                bPressed = true;
            }
            while (bPressed == true) {
                glyphMotor1.setPower(1.0);
                glyphMotor2 .setPower(1.0);
                glyphMotor3.setPower(1.0);
                glyphMotor4.setPower(1.0);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
