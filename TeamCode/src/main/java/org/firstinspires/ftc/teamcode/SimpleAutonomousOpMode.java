package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
@Autonomous(name = "Simple Autonomous Test", group = "Linear OpMode")
public class SimpleAutonomousOpMode extends LinearOpMode {

    // Expansion Hub Motors (4 Drive Motors)
    private DcMotor leftFrontEx  = null;
    private DcMotor leftBackEx   = null;
    private DcMotor rightFrontEx = null;
    private DcMotor rightBackEx  = null;

    // Control Hub Motors (4 Extra Motors)
    private DcMotor motorFrontLeft  = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorRearLeft   = null;
    private DcMotor motorRearRight  = null;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize Expansion Hub Motors using your exact names
        leftFrontEx  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackEx   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontEx = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackEx  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize Control Hub Motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorRearLeft   = hardwareMap.get(DcMotor.class, "motorRearLeft");
        motorRearRight  = hardwareMap.get(DcMotor.class, "motorRearRight");

        // Set directions
        leftFrontEx.setDirection(DcMotor.Direction.REVERSE);
        leftBackEx.setDirection(DcMotor.Direction.REVERSE);
        rightFrontEx.setDirection(DcMotor.Direction.FORWARD);
        rightBackEx.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Diagnostic Setup: Ready to test each motor one by one");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // =======================================================
        // EXPANSION HUB MOTORS
        // =======================================================
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("leftFrontEx (left_front_drive)", leftFrontEx);
        }
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("leftBackEx (left_back_drive)", leftBackEx);
        }
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("rightFrontEx (right_front_drive)", rightFrontEx);
        }
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("rightBackEx (right_back_drive)", rightBackEx);
        }

        // =======================================================
        // CONTROL HUB MOTORS
        // =======================================================
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("motorFrontLeft (motorFrontleft)", motorFrontLeft);
        }
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("motorFrontRight (motorFrontRight)", motorFrontRight);
        }
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("motorRearLeft (motorRearLeft)", motorRearLeft);
        }
        if (opModeIsActive()) {
            runSingleMotorDiagnostic("motorRearRight (motorRearRight)", motorRearRight);
        }

        // Diagnostic Completed
        stopAllMotors();
        telemetry.addData("Status", "Diagnostic Complete! Check which motor didn't spin.");
        telemetry.update();
        sleep(3000);
    }

    /**
     * Helper method to isolate and run a single motor for diagnostic testing.
     */
    private void runSingleMotorDiagnostic(String motorName, DcMotor motor) {
        runtime.reset();
        
        // Command only this motor to move
        motor.setPower(0.4);
        
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("DIAGNOSTIC ACTIVE", "Testing single motor...");
            telemetry.addData("Testing Now", motorName);
            telemetry.addData("Time Remaining", "%.1f s", 2.0 - runtime.seconds());
            telemetry.update();
        }
        
        // Stop it before moving to the next step
        motor.setPower(0);
        sleep(500); // 0.5 second pause between steps
    }

    /**
     * Instantly stops power to all 8 configured motors
     */
    private void stopAllMotors() {
        if (leftFrontEx != null) leftFrontEx.setPower(0);
        if (leftBackEx != null) leftBackEx.setPower(0);
        if (rightFrontEx != null) rightFrontEx.setPower(0);
        if (rightBackEx != null) rightBackEx.setPower(0);
        
        if (motorFrontLeft != null) motorFrontLeft.setPower(0);
        if (motorFrontRight != null) motorFrontRight.setPower(0);
        if (motorRearLeft != null) motorRearLeft.setPower(0);
        if (motorRearRight != null) motorRearRight.setPower(0);
    }
}
