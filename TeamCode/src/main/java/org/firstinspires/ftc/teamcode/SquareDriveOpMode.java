package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
//@Autonomous(name = "Drive in 1m Square Box", group = "Linear OpMode")
public class SquareDriveOpMode extends LinearOpMode {

    // Define wheels on the Control Hub
    private DcMotor motorFrontLeft  = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorRearLeft   = null;
    private DcMotor motorRearRight  = null;

    private final ElapsedTime runtime = new ElapsedTime();

    // Adjust these timing estimates based on your robot weight, battery level, and wheel grip!
    // Approximately how long it takes to move 1 meter forward at 40% power
    private static final double DRIVE_TIME_1_METER = 2.2; 
    // Approximately how long it takes to make a perfect 90-degree pivot turn at 40% power
    private static final double TURN_TIME_90_DEG   = 1.1; 

    @Override
    public void runOpMode() {
        // Initialize Control Hub drive motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorRearLeft   = hardwareMap.get(DcMotor.class, "motorRearLeft");
        motorRearRight  = hardwareMap.get(DcMotor.class, "motorRearRight");

        // Set directions so pushing positive power moves the wheels forward
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Ready to drive in a 1-meter square box!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Loop 4 times to complete the 4 sides of a square box path
        for (int side = 1; side <= 4 && opModeIsActive(); side++) {
            
            // ----------------------------------------------------
            // SIDE PATH: Move forward 1 Meter
            // ----------------------------------------------------
            runtime.reset();
            setDrivePower(0.4, 0.4, 0.4, 0.4); // Run all wheels forward at 40% power
            
            while (opModeIsActive() && (runtime.seconds() < DRIVE_TIME_1_METER)) {
                telemetry.addData("Square Side", "%d of 4", side);
                telemetry.addData("Status", "Moving Forward 1 Meter");
                telemetry.addData("Elapsed", "%.1f / %.1f s", runtime.seconds(), DRIVE_TIME_1_METER);
                telemetry.update();
            }

            // Pause briefly to clear momentum before turning
            stopRobot();
            sleep(400);

            // ----------------------------------------------------
            // CORNER TURN: Pivot 90 Degrees Right
            // ----------------------------------------------------
            if (opModeIsActive()) {
                runtime.reset();
                // Left wheels turn forward, right wheels turn backward to spin right in place
                setDrivePower(0.4, -0.4, 0.4, -0.4);
                
                while (opModeIsActive() && (runtime.seconds() < TURN_TIME_90_DEG)) {
                    telemetry.addData("Square Side", "%d of 4", side);
                    telemetry.addData("Status", "Turning 90 Degrees Right");
                    telemetry.addData("Elapsed", "%.1f / %.1f s", runtime.seconds(), TURN_TIME_90_DEG);
                    telemetry.update();
                }

                // Pause briefly to clear momentum before driving the next side
                stopRobot();
                sleep(400);
            }
        }

        // Routine finished
        stopRobot();
        telemetry.addData("Status", "Square box path sequence complete!");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Helper method to assign power levels to all 4 wheels
     */
    private void setDrivePower(double fl, double fr, double rl, double rr) {
        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorRearLeft.setPower(rl);
        motorRearRight.setPower(rr);
    }

    /**
     * Instantly cuts power to all wheels
     */
    private void stopRobot() {
        setDrivePower(0, 0, 0, 0);
    }
}
