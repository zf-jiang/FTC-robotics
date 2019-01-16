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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Vuforia;

// Using REV IMU for robot orientation
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// Using Vuforia for sample detection
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

/* VUFORIA KEY:
ATsQBJ//////AAABmWTX7RRHP0rNtHOPB8fJqA8BK0Sxs7V8T8w1X7GKyV3W4JP72rEQHIaor5bYEMC2WNnVQOhU0l1yZGynqMyilPgemnYL/mjLZlag95PHc+85Qbo0YvSmHrLWwciBneUbMDcN+LAjUJt+ziY6cW52fHG1ID/r7qZJZJ+DQQly94h9AscUQcONOFKv3Cnv0/WX+IVb1VyhHn8aHKbLK/+2V+LewA1I3jw8wJ72g0KTAolCPWmtk1st5WQRcuTlZHCqt55HXa7ih8kTeiLzQmOE2hRmATzQku6L+Bud3VMoSIJ+4AUoIRXZ1C04ic7NkkKDbgsE5dRz5HyBvEtlReA/+qGeKy2amqXTdHjlNknwa7lq
 */

@Autonomous(name="TestAutonomous", group="TestAutonomous") // @Autonomous
//@Disabled
public class TestAutonomous extends LinearOpMode {

    // Declare DCMotors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    // Declare IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double power = .30;
    //double correction;

    // Declare Vuforia
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Autonomous Initialized");
        telemetry.update();

        // Hardware initialization
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        backRightMotor = hardwareMap.get(DcMotor.class, "RR");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // Calibrate IMU
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Vuforia initialization
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "ATsQBJ//////AAABmWTX7RRHP0rNtHOPB8fJqA8BK0Sxs7V8T8w1X7GKyV3W4JP72rEQHIaor5bYEMC2WNnVQOhU0l1yZGynqMyilPgemnYL/mjLZlag95PHc+85Qbo0YvSmHrLWwciBneUbMDcN+LAjUJt+ziY6cW52fHG1ID/r7qZJZJ+DQQly94h9AscUQcONOFKv3Cnv0/WX+IVb1VyhHn8aHKbLK/+2V+LewA1I3jw8wJ72g0KTAolCPWmtk1st5WQRcuTlZHCqt55HXa7ih8kTeiLzQmOE2hRmATzQku6L+Bud3VMoSIJ+4AUoIRXZ1C04ic7NkkKDbgsE5dRz5HyBvEtlReA/+qGeKy2amqXTdHjlNknwa7lq";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //runtime.reset();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.
        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    aimLeftMineral();
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    aimMiddleMineral();
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    aimRightMineral();
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    // Just drive forward and hope goldPosition = CENTER;
                    aimMiddleMineral();
                    break;
            }

            telemetry.update();
            // idle(); To be called at the bottom of while (opModeIsActive()) loop
        }
        vision.shutdown();
    }

    // Resets the cumulative angle tracking to zero
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Get current cumulative angle rotation from last reset
    // @return Angle in degrees... + is left, - is right
    private double getAngle() {
        /*
         * Determine if Z axis is the axis to use for heading angle...
         * Must process the angle because the imu works in euler angles so the Z axis is
         * returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
         * 180 degrees...
         * Detect this transition and track the total cumulative angle of rotation
         */

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    /* NOTICE: I don't think this works for us because our movement is holonomic...probably needs adjusting.
    // See if moving in a straight line and if not return a power correction value
    // @return Power adjustment, + is adjust left, - is adjust right

    private double checkDirection() {
         * The gain value determines how sensitive the correction is to direction changes...
         * Have to experiment to get small smooth direction changes to stay on a straight line

        double correction, angle, gain = .10;
        angle = getAngle();

        if (angle == 0) {
            // No adjustment
            correction = 0;
        }
        else {
            // Reverse sign of angle for correction
            correction = -angle;
        }

        correction = correction * gain;
        return correction;
    }
     */

     // Rotate left or right the number of degrees...does not support turning more than 180 degrees
     // @param degrees Degrees to turn, + is left - is right
    private void rotate(int degrees, double power) {
        // Restart imu movement tracking
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).
        if (degrees < 0) {
            while (opModeIsActive() && (getAngle() == 0 || getAngle() > degrees) {
                turnRight(power);
            }
        }
        else {
            while (opModeIsActive() && getAngle() < degrees) {
                turnLeft(power);
            }
        }

        // Turn the motors off
        stopDriving();

        // Wait for rotation to stop
        sleep(1000);

        // Reset angle tracking on new heading
        resetAngle();
    }

    // Set power to wheel
    public void powerToWheels(double flPower, double frPower, double blPower, double brPower) {
        frontLeftMotor.setPower(flPower);
        frontRightMotor.setPower(frPower);
        backLeftMotor.setPower(blPower);
        backRightMotor.setPower(brPower);
    }

    public void turnLeft(double power) {
        powerToWheels(-power, power, -power, power);
    }

    public void turnRight(double power) {
        powerToWheels(power, -power, power, -power);
    }

    public void strafeLeft(double power) {
        powerToWheels(-power, power, power, -power);
    }

    public void  strafeRight(double power) {
        powerToWheels(power, -power, -power, power);
    }

    public void driveForward(double power) {
        powerToWheels(power, power, power, power);
    }

    public void driveBackward(double power) {
        powerToWheels(-power, -power, -power, -power);
    }

    public void stopDriving(){
        powerToWheels(0, 0, 0, 0);
    }

    public void aimLeftMineral() {
        // Turn left a certain degrees
        // requires testing
    }

    public void aimRightMineral() {
        // Turn right a certain degrees
        // requires testing
    }

    public void aimMiddleMineral() {
        // Do we need any turning at all?
        // requires testing
    }

    public void raiseMarkerArm(){

    }

    public void lowerMarkerArm(){

    }
}
