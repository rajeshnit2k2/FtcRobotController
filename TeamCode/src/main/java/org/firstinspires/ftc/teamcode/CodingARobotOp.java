package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Drives a two-wheel (rear left + rear right) robot and moves a servo arm, all from gamepad1.
 *
 * Controls:
 *   Left stick Y  - drive forward / backward
 *   Right stick X - turn left / right
 *   Right trigger - raise the arm (press harder = faster)
 *   Left trigger  - lower the arm (press harder = faster)
 *   A             - snap arm to the DOWN position
 *   Y             - snap arm to the UP position
 *   B             - snap arm to the MIDDLE position
 */
@TeleOp(name = "Coding A Robot OpMode", group = "Linear OpMode")
public class CodingARobotOp extends LinearOpMode {

    // Hardware config names, as set on the Driver Station robot configuration
    private static final String LEFT_MOTOR_NAME  = "motorRearLeft";
    private static final String RIGHT_MOTOR_NAME = "motorRearRight";
    private static final String ARM_SERVO_NAME   = "armServo";

    // Caps the drive output so full joystick deflection is 60% power
    private static final double MAX_SPEED = 0.6;

    // Ignore tiny stick deflection so the robot doesn't creep when the sticks are at rest
    private static final double STICK_DEADZONE = 0.05;

    // Arm servo positions (0.0 to 1.0). Tune these to match how the arm is mounted.
    private static final double ARM_DOWN   = 0.0;
    private static final double ARM_MIDDLE = 0.5;
    private static final double ARM_UP     = 1.0;

    // Largest amount the arm position can move in a single loop, at full trigger press
    private static final double ARM_MAX_STEP_PER_LOOP = 0.01;

    private DcMotor motorRearLeft  = null;
    private DcMotor motorRearRight = null;
    private Servo   armServo       = null;

    @Override
    public void runOpMode() {
        motorRearLeft  = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME);
        motorRearRight = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME);
        armServo       = hardwareMap.get(Servo.class, ARM_SERVO_NAME);

        // Set directions so pushing positive power moves the wheels forward
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);

        // Hold position instead of coasting when the sticks are released
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start the arm in a known, safe position
        double armPosition = ARM_MIDDLE;
        armServo.setPosition(armPosition);

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- Drive ----------------
            double drive = applyDeadzone(-gamepad1.left_stick_y);
            double turn  = applyDeadzone(gamepad1.right_stick_x);

            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Normalize so the combined inputs never exceed full scale
            double maxMagnitude = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftPower  = leftPower  / maxMagnitude * MAX_SPEED;
            rightPower = rightPower / maxMagnitude * MAX_SPEED;

            motorRearLeft.setPower(leftPower);
            motorRearRight.setPower(rightPower);

            // ---------------- Arm ----------------
            if (gamepad1.a) {
                armPosition = ARM_DOWN;
            } else if (gamepad1.y) {
                armPosition = ARM_UP;
            } else if (gamepad1.b) {
                armPosition = ARM_MIDDLE;
            } else {
                // Triggers move the arm gradually; how hard each is pressed sets the speed
                double armInput = gamepad1.right_trigger - gamepad1.left_trigger;
                armPosition += armInput * ARM_MAX_STEP_PER_LOOP;
            }
            armPosition = Math.max(ARM_DOWN, Math.min(ARM_UP, armPosition));
            armServo.setPosition(armPosition);

            telemetry.addData("Status", "Running");
            telemetry.addData("Drive/Turn", "Drive: %.2f, Turn: %.2f", drive, turn);
            telemetry.addData("Motor Power", "Left: %.2f, Right: %.2f", leftPower, rightPower);
            telemetry.addData("Arm Position", "%.3f", armPosition);
            telemetry.update();
        }
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
