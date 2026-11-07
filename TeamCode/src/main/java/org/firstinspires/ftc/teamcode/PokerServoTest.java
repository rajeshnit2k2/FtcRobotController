package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Bare-bones test for the arm (pokerServo). Nothing else on the robot is touched.
 *
 * Controls (gamepad1, after START):
 *   A - go to 0.0      B - go to 0.5      Y - go to 1.0
 *   D-pad up/down - nudge position by 0.02
 *
 * If the "Commanded" number changes but the servo never moves, the problem is wiring/port/config,
 * not the TeleOp code.
 */
//@TeleOp(name = "Poker Servo Test", group = "Test")
public class PokerServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "pokerServo");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.addData("Port", servo.getPortNumber());
        telemetry.addData("Controller", servo.getConnectionInfo());
        telemetry.addLine("Press START, then A / B / Y");
        telemetry.update();
        waitForStart();

        servo.setPwmEnable();
        double position = 0.5;
        boolean lastUp = false, lastDown = false;

        while (opModeIsActive()) {
            if (gamepad1.a) position = 0.0;
            if (gamepad1.b) position = 0.5;
            if (gamepad1.y) position = 1.0;
            if (gamepad1.dpad_up && !lastUp) position += 0.02;
            if (gamepad1.dpad_down && !lastDown) position -= 0.02;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            position = Math.max(0.0, Math.min(1.0, position));

            servo.setPosition(position);

            telemetry.addData("Commanded", "%.2f  (~%.0f deg)", position, position * 270);
            telemetry.addData("PWM enabled", servo.isPwmEnabled());
            telemetry.addData("Port", servo.getPortNumber());
            telemetry.addData("Controller", servo.getConnectionInfo());
            telemetry.update();
        }
    }
}
