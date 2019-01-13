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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

/*
VUFORIA KEY:
ATsQBJ//////AAABmWTX7RRHP0rNtHOPB8fJqA8BK0Sxs7V8T8w1X7GKyV3W4JP72rEQHIaor5bYEMC2WNnVQOhU0l1yZGynqMyilPgemnYL/mjLZlag95PHc+85Qbo0YvSmHrLWwciBneUbMDcN+LAjUJt+ziY6cW52fHG1ID/r7qZJZJ+DQQly94h9AscUQcONOFKv3Cnv0/WX+IVb1VyhHn8aHKbLK/+2V+LewA1I3jw8wJ72g0KTAolCPWmtk1st5WQRcuTlZHCqt55HXa7ih8kTeiLzQmOE2hRmATzQku6L+Bud3VMoSIJ+4AUoIRXZ1C04ic7NkkKDbgsE5dRz5HyBvEtlReA/+qGeKy2amqXTdHjlNknwa7lq
 */

@Autonomous(name="TestAutonomous", group="TestAutonomous") // @Autonomous
//@Disabled
public class TestAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Autonomous Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        backRightMotor = hardwareMap.get(DcMotor.class, "RR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
                    // code goes here
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    // code goes here
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    // code goes here
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    // code goes here
                    break;
            }

            telemetry.update();
            // idle(); To be called at the bottom of while (opModeIsActive()) loop
        }
        vision.shutdown();
    }
}
