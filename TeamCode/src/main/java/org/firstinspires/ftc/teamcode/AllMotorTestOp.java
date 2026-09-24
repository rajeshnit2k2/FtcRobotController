package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/*
 * Drives the mecanum base and runs every other motor on the robot.
 *
 * Controls (gamepad1):
 *   Left stick Y   - drive forward / backward
 *   Left stick X   - strafe east / west (right / left)
 *   Right stick X  - turn left / right
 *   R2 (trigger)   - run all the other (non-drive) motors
 */
@TeleOp(name = "All Motor Test", group = "Linear OpMode")
public class AllMotorTestOp extends LinearOpMode {

    // Configuration names of the four drive motors
    private static final String FRONT_LEFT  = "motorFrontLeft";
    private static final String FRONT_RIGHT = "motorFrontRight";
    private static final String REAR_LEFT   = "motorRearLeft";
    private static final String REAR_RIGHT  = "motorRearRight";

    // Caps the output so full joystick / trigger deflection is 60% power
    private static final double MAX_SPEED = 1.0;

    private DcMotor motorFrontLeft  = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorRearLeft   = null;
    private DcMotor motorRearRight  = null;

    // Every motor in the configuration that is not a drive motor
    private final List<String>  otherNames  = new ArrayList<>();
    private final List<DcMotor> otherMotors = new ArrayList<>();

    @Override
    public void runOpMode() {
        motorFrontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT);
        motorFrontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT);
        motorRearLeft   = hardwareMap.get(DcMotor.class, REAR_LEFT);
        motorRearRight  = hardwareMap.get(DcMotor.class, REAR_RIGHT);

        // Set directions so pushing positive power moves the wheels forward
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);

        List<String> driveNames = Arrays.asList(FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT);
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            if (!driveNames.contains(entry.getKey())) {
                otherNames.add(entry.getKey());
                otherMotors.add(entry.getValue());
            }
        }

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.addData("Other motors (R2)", otherNames.isEmpty() ? "none found" : otherNames.toString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Left stick Y drives forward/back, left stick X strafes east/west,
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

            frontLeftPower  = frontLeftPower  / maxMagnitude * MAX_SPEED;
            rearLeftPower   = rearLeftPower   / maxMagnitude * MAX_SPEED;
            frontRightPower = frontRightPower / maxMagnitude * MAX_SPEED;
            rearRightPower  = rearRightPower  / maxMagnitude * MAX_SPEED;

            motorFrontLeft.setPower(frontLeftPower);
            motorRearLeft.setPower(rearLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorRearRight.setPower(rearRightPower);

            // R2 runs every non-drive motor; power follows how far it is pressed
            double otherPower = gamepad1.right_trigger * MAX_SPEED;
            for (DcMotor motor : otherMotors) {
                motor.setPower(otherPower);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Drive/Strafe/Turn", "Drive: %.2f, Strafe: %.2f, Turn: %.2f", drive, strafe, turn);
            telemetry.addData("Motor Power", "FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
                    frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
            telemetry.addData("Other motors (R2)", "%s at %.2f", otherNames, otherPower);
            telemetry.update();
        }
    }
}