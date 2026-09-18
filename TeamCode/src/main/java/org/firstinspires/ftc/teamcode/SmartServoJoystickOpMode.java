package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Drives a REV Smart Robot Servo (or any standard positional servo) entirely from a
 * single gamepad's joystick.
 *
 * Controls:
 *   Left stick X - direction AND speed of rotation. How far you push the stick left or
 *                  right sets how fast the servo sweeps that way; centered stick = stopped.
 *   Left bumper  - snap/hold the servo at its minimum position
 *   Right bumper - snap/hold the servo at its maximum position
 *
 * A positional servo has no native "speed" input, only a target position (0.0 to 1.0).
 * To get joystick-style continuous rotation with variable speed, this OpMode sweeps the
 * target position a little further each loop, where both the direction and the step size
 * come from how far the stick is pushed.
 */
@TeleOp(name = "Smart Servo Joystick OpMode", group = "Linear OpMode")
public class SmartServoJoystickOpMode extends LinearOpMode {

    // Hardware config name for the servo, as set on the Driver Station robot configuration
    private static final String SERVO_NAME = "smart_servo";

    // Ignore tiny stick deflection so the servo doesn't creep when the stick is at rest
    private static final double STICK_DEADZONE = 0.05;

    // Largest amount the servo position can move in a single loop, at full trigger speed
    private static final double MAX_STEP_PER_LOOP = 0.02;

    // REV Smart Robot Servo's usable position range (0.0 to 1.0) maps to about 300 degrees
    private static final double SERVO_DEGREES_RANGE = 300.0;

    private Servo servo = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variable. The name here must match the servo's name
        // in the robot configuration on the Driver Station / Robot Controller.
        servo = hardwareMap.get(Servo.class, SERVO_NAME);

        // Start centered so the first loop has a known, safe position
        double servoPosition = 0.5;
        servo.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double stickInput = gamepad1.left_stick_x;
            if (Math.abs(stickInput) < STICK_DEADZONE) {
                stickInput = 0.0;
            }

            if (gamepad1.left_bumper) {
                // Snap directly to minimum position
                servoPosition = 0.0;
            } else if (gamepad1.right_bumper) {
                // Snap directly to maximum position
                servoPosition = 1.0;
            } else {
                // Sweep the target position each loop; how far the stick is pushed sets both
                // the direction (sign) and the speed (magnitude) of that sweep.
                servoPosition += stickInput * MAX_STEP_PER_LOOP;
                servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
            }

            servo.setPosition(servoPosition);

            telemetry.addData("Status", "Running");
            telemetry.addData("Stick Input", "%.2f", stickInput);
            telemetry.addData("Servo Position", "%.3f", servoPosition);
            telemetry.addData("Servo Angle", "%.1f deg", servoPosition * SERVO_DEGREES_RANGE);
            telemetry.update();
        }
    }
}
