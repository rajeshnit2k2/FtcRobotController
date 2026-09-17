package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings("unused")
//@TeleOp(name = "Simple Motor OpMode", group = "Linear OpMode")
public class SimpleMotorOpMode extends LinearOpMode {

    // Declare motor objects
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step on the FTC Driver Station / Robot Controller.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Use the left joystick Y axis to control the left motor power,
            // and the right joystick Y axis to control the right motor power (Tank Drive).
            // Joysticks give a negative value when pushed forward, so we negate it.
            double leftPower  = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            // Set power to the motors
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Send telemetry messages to the Driver Station
            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power", "Left: %.2f, Right: %.2f", leftPower, rightPower);
            telemetry.update();
        }
    }
}
