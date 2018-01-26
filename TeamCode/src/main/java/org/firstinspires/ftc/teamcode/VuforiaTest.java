/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Vuforia Test 1.0", group ="Autonomous")

public class VuforiaTest extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1220;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;
    static final double SERVO_DOWN = 0.50; // ** CLOSED **
    static final double SERVO_UP = 0.9; // ** OPEN **
    public DcMotor motorFLeft = null;
    public DcMotor motorFRight = null;
    public DcMotor motorBLeft = null; //set motors to nothing
    public DcMotor motorBRight = null; //set motors to nothing
    public ColorSensor colorSensor = null;
    public Servo colorServo = null;
    public DcMotor motorArm = null;
    public Servo servoArm = null;
    public Servo servoArm2 = null;
    public boolean redTeam;
    public boolean blueTeam;
    public boolean confirmation = false;
    public String decision;
    //public ServoControllerEx servoControl = null;
    public final int SERVO_PORT = 2;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    VuforiaTrackable relicTemplate;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AW7cBtn/////AAAAGYMkfBwkBk+RtHulA/YtafKMtOkKRJGL0" +
                "uSzzGVGQDzIzhkvOQY4lJnPIwwP7MBF5k8AKuhF0QwYx4nlBgTOlMv23OSHNfvy9tE0Egigzp" +
                "bi7p0zUS3bgP9XPd8IsdyQQhwdQQFY64eZiNxqstMPQqOhCyZ+MuQjWWiW1gAeGaPNxD8sUUS" +
                "FbcP0F0LfTWY8JYNFDjr7vUfB8koqwsWCYrB8gQH9ZAwVRDQlHbpx4z1ZEySLnDCRXX0Ns4R1" +
                "PktDJeHJq4Xj0wInNVNPLwCSXaw/yLDkPGa+IWws63uWwr3h4TaxJEs4zgqqbTyCVDPgeAtjm" +
                "wT6KCrrlsS6kW1czPF8bNWq4U5BdinbKBZGrKS7";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); //can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        motorBLeft = hardwareMap.get(DcMotor.class, "motorBLeft");
        motorBRight = hardwareMap.get(DcMotor.class, "motorBRight");
        motorFLeft = hardwareMap.get(DcMotor.class, "motorFLeft");
        motorFRight = hardwareMap.get(DcMotor.class, "motorFRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorServo = hardwareMap.get(Servo.class, "colorServo");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoArm2 = hardwareMap.get(Servo.class, "servoArm2");

        motorFLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFRight.setDirection(DcMotor.Direction.FORWARD);
        motorBLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBRight.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
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
        relicTrackables.activate();

        while (opModeIsActive()) {
            servoArm.setPosition(SERVO_DOWN);
            servoArm2.setPosition(1.0 - SERVO_DOWN);
            colorServo.setPosition(1.0);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                encoderDrive(DRIVE_SPEED, 5, 5, 3.0);
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                encoderDrive(DRIVE_SPEED, -5, -5, 3.0);
            }
            else if (vuMark == RelicRecoveryVuMark.LEFT) {
                encoderDrive(DRIVE_SPEED, 5, -5, 3.0);
            }

            telemetry.addLine("RIGHT!!!, RIGHT!!!");
            telemetry.update();
        }

    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    //**ENCODER DRIVE**
    public void encoderDrive(double speed, double leftInches, double rightInches, double time) {
        int motorFLeftTarget;
        int motorFRightTarget;
        int motorBLeftTarget;
        int motorBRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            motorFLeftTarget = motorFLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            motorFRightTarget = motorFRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorBLeftTarget = motorBLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            motorBRightTarget = motorBRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            motorFLeft.setTargetPosition(motorFLeftTarget);
            motorFRight.setTargetPosition(motorFRightTarget);
            motorBLeft.setTargetPosition(motorBLeftTarget);
            motorBRight.setTargetPosition(motorBRightTarget);

            // Turn On RUN_TO_POSITION
            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFLeft.setPower(speed);
            motorFRight.setPower(speed);
            motorBLeft.setPower(speed);
            motorBRight.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < time) && (motorFLeft.isBusy() && motorFRight.isBusy() && motorBLeft.isBusy() && motorBRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", motorFLeftTarget,  motorFRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", motorFLeft.getCurrentPosition(), motorFRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFLeft.setPower(0);
            motorFRight.setPower(0);
            motorBLeft.setPower(0);
            motorBRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    } //**ENCODER DRIVE**
}
