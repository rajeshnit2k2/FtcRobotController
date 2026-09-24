package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Joystick Drive OpMode", group = "Linear OpMode")
public class JoystickDriveOpMode extends LinearOpMode {

    // Control Hub drive motors
    private DcMotor motorFrontLeft  = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorRearLeft   = null;
    private DcMotor motorRearRight  = null;

    // Caps the drive output so full joystick deflection is 60% power
    private static final double MAX_SPEED = 0.6;

    @Override
    public void runOpMode() {
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorRearLeft   = hardwareMap.get(DcMotor.class, "motorRearLeft");
        motorRearRight  = hardwareMap.get(DcMotor.class, "motorRearRight");

        // Set directions so pushing positive power moves the wheels forward
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Left stick Y drives forward/back, left stick X turns left/right
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.left_stick_x;

            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Normalize so combined drive+turn never exceeds full scale
            double maxMagnitude = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMagnitude > 1.0) {
                leftPower  /= maxMagnitude;
                rightPower /= maxMagnitude;
            }

            // Scale down so max output is 60% power
            leftPower  *= MAX_SPEED;
            rightPower *= MAX_SPEED;

            motorFrontLeft.setPower(leftPower);
            motorRearLeft.setPower(leftPower);
            motorFrontRight.setPower(rightPower);
            motorRearRight.setPower(rightPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Drive/Turn", "Drive: %.2f, Turn: %.2f", drive, turn);
            telemetry.addData("Motor Power", "Left: %.2f, Right: %.2f", leftPower, rightPower);
            telemetry.update();
        }
    }
}
