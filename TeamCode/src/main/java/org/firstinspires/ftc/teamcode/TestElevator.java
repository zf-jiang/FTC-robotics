package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestElevator", group="TestElevator")
//@Disabled
public class TestElevator extends OpMode {

    // Declare OpMode members.
    // 4WD Mecanum wheel drive base
    private DcMotor elevatorMotor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        elevatorMotor  = hardwareMap.get(DcMotor.class, "elevator");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "All motors have been mapped");
    }

    public void loop() {
        double motorPower = 1.0;
        telemetry.addData("Status", "Testing motor power");
        elevatorMotor.setPower(motorPower);
        String msg = String.format("Power for all motors set to %s", motorPower);
        telemetry.addData("Status", msg);
    }
}
