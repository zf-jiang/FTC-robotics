package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TestMotorPower", group="TestMotorPower")
//@Disabled
public class TestMotorPower extends OpMode {

    // Declare OpMode members.
    // 4WD Mecanum wheel drive base
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftMotor = hardwareMap.get(DcMotor.class, "rear_left_drive");
        backRightMotor = hardwareMap.get(DcMotor.class, "rear_right_drive");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "All motors have been mapped");

        int flPosition = frontLeftMotor.getCurrentPosition();
        int frPosition = frontRightMotor.getCurrentPosition();
        int blPosition = backLeftMotor.getCurrentPosition();
        int brPosition = backRightMotor.getCurrentPosition();
    }

    public void loop() {
        double motorPower = 1.0;
        telemetry.addData("Status", "Testing motor power");
        frontLeftMotor.setPower(motorPower);
        frontRightMotor.setPower(motorPower);
        backLeftMotor.setPower(motorPower);
        backRightMotor.setPower(motorPower);
        String msg = String.format("Power for all motors set to %s", motorPower);
        telemetry.addData("Status", msg);
    }
}
