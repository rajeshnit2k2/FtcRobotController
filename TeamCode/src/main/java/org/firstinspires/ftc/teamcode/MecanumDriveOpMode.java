package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum Drive OpMode", group = "Linear OpMode")
public class MecanumDriveOpMode extends LinearOpMode {

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

            // Left stick Y drives forward/back, left stick X strafes left/right,
            // right stick X turns left/right
            double drive  = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn   = gamepad1.right_stick_x;

            double frontLeftPower  = drive + strafe + turn;
            double rearLeftPower   = drive - strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double rearRightPower  = drive + strafe - turn;

            // Normalize so the combined inputs never exceed full scale
            double maxMagnitude = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(rearLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));

            frontLeftPower  /= maxMagnitude;
            rearLeftPower   /= maxMagnitude;
            frontRightPower /= maxMagnitude;
            rearRightPower  /= maxMagnitude;

            // Scale down so max output is 60% power
            frontLeftPower  *= MAX_SPEED;
            rearLeftPower   *= MAX_SPEED;
            frontRightPower *= MAX_SPEED;
            rearRightPower  *= MAX_SPEED;

            motorFrontLeft.setPower(frontLeftPower);
            motorRearLeft.setPower(rearLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorRearRight.setPower(rearRightPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Drive/Strafe/Turn", "Drive: %.2f, Strafe: %.2f, Turn: %.2f", drive, strafe, turn);
            telemetry.addData("Motor Power", "FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
                    frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
            telemetry.update();
        }
    }
}
