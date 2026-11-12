package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BIOBUZZ (2026-2027) Autonomous for the REV DUO FTC Starter Bot. Runs in the 30 second AUTO period.
 *
 * Plan:
 *   1. (Optional) wait a few seconds so we don't collide with our alliance partner.
 *   2. Spin up the launcher, then LAUNCH the 4 pre-loaded POLLEN straight ahead into our upward-facing
 *      CELL (counts toward a HIVE TIP, 20 pts).
 *   3. Drive off the wall for LEAVE (3 pts).
 *   4. (Optional) drive into a LOADING ZONE for PARK (5 pts):
 *        NEAR (BiobuzzAutoNear): turn and drive into the closest LOADING ZONE.
 *        FAR  (BiobuzzAutoFar):  L-shaped path around the HIVE to the diagonally opposite LOADING ZONE.
 *
 * Setup (G304): place the robot on your own half of the FIELD, touching the perimeter wall, outside the
 * LOADING ZONE, touching its 4 pre-loaded POLLEN, with the launcher aimed at your alliance's upward-facing CELL.
 *
 * No camera / AprilTags needed. During INIT, use gamepad1 to choose options (shown on the Driver Station):
 *   X / B            = alliance BLUE / RED (mirrors all PARK turns)
 *   D-pad up/down    = start delay +/- 1 second
 *   D-pad left/right = PARK off/on
 *
 * Driving uses the drive motor encoders, so moves are in inches and degrees. Tune the *_INCHES and
 * *_DEGREES constants on a practice field. Tune turns with TRACK_WIDTH_INCHES (bigger = turns further).
 */
public abstract class BiobuzzAuto extends LinearOpMode {

    // Launcher settings, shared with TeleOp so both shoot the same
    static final double LAUNCHER_TARGET_VELOCITY = BiobuzzTeleOp.LAUNCHER_TARGET_VELOCITY;
    static final double LAUNCHER_READY_TOLERANCE = BiobuzzTeleOp.LAUNCHER_READY_TOLERANCE;
    // Longest we wait for the launcher to reach speed before shooting anyway
    static final double LAUNCHER_SPINUP_TIMEOUT_SECONDS = 3.0;
    // Long enough to push all 4 pre-loads through the launcher
    static final double FEED_SECONDS = 4.0;

    // ---------------- Encoder driving ----------------
    // REV HD Hex Motor (28 ticks/rev) + 20:1 UltraPlanetary (real ratio 3.61 x 5.23 = 18.88), 90 mm wheels
    static final double DRIVE_TICKS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_RATIO = 3.61 * 5.23;
    static final double WHEEL_DIAMETER_INCHES = 90 / 25.4;
    static final double TICKS_PER_INCH =
            DRIVE_TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);
    // Distance between the left and right wheels. Used to turn by degrees.
    // If a 90 degree turn comes up short, make this bigger; if it overshoots, make it smaller.
    static final double TRACK_WIDTH_INCHES = 16.0;
    // Give up on a move after this long (e.g. robot is pushed against something) so AUTO keeps going
    static final double MOVE_TIMEOUT_SECONDS = 4.0;

    // Positive drive power moves the robot toward its NON-launcher end, so straight drives are reversed to
    // move toward the launcher end (away from the wall). Turns are not affected.
    // Set to +1 if the robot ever drives the wrong way on LEAVE.
    static final double DRIVE_DIRECTION = -1;

    static final double DRIVE_POWER = 0.5;
    static final double TURN_POWER = 0.4;

    // LEAVE: drive straight away from the wall (both NEAR and FAR)
    static final double LEAVE_INCHES = 12;

    // Turn angles are for BLUE and mirrored automatically on RED.
    // Positive degrees = the left wheel drives forward (robot turns right). Flip the sign if a turn goes the
    // wrong way.

    // NEAR PARK: turn toward the closest LOADING ZONE, then drive into it
    static final double NEAR_TURN_DEGREES = 90;
    static final double NEAR_PARK_INCHES = 24;

    // FAR PARK (L-shaped, around the HIVE): turn away from the near zone, drive along the field past the
    // HIVE, then turn and drive into the diagonally opposite LOADING ZONE
    static final double FAR_TURN1_DEGREES = -90;
    static final double FAR_CROSS_INCHES = 72;
    static final double FAR_TURN2_DEGREES = 90;
    static final double FAR_PARK_INCHES = 24;

    // Keep this low enough that the FAR park still finishes inside the 30 s AUTO
    static final int MAX_START_DELAY_SECONDS = 15;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor intakeMotor;
    private DcMotorEx launcherMotor;
    private CRServo feederServo;

    private final boolean parkFar;
    private boolean isBlue = true;
    private boolean parkEnabled = true;
    private int startDelaySeconds;

    private final ElapsedTime timer = new ElapsedTime();

    /** @param parkFar true = park in the diagonally opposite LOADING ZONE, false = the closest one */
    protected BiobuzzAuto(boolean parkFar) {
        this.parkFar = parkFar;
    }

    @Override
    public void runOpMode() {
        initHardware();
        selectOptions();

        // selectOptions() returns on START or STOP; bail out if it was STOP
        if (!opModeIsActive()) {
            return;
        }

        waitSeconds(startDelaySeconds);
        launchPreloads();
        driveInches(LEAVE_INCHES);

        if (parkEnabled) {
            if (parkFar) {
                turnDegrees(FAR_TURN1_DEGREES);
                driveInches(FAR_CROSS_INCHES);
                turnDegrees(FAR_TURN2_DEGREES);
                driveInches(FAR_PARK_INCHES);
            } else {
                turnDegrees(NEAR_TURN_DEGREES);
                driveInches(NEAR_PARK_INCHES);
            }
        }

        stopAll();
        telemetry.addData("Status", "AUTO complete");
        telemetry.update();
    }

    private void initHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");

        // Keep these in sync with BiobuzzTeleOp
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);
        feederServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Lets the drive team pick alliance, delay and park while waiting for START. Robot stays still (G304.H).
     */
    private void selectOptions() {
        boolean lastUp = false, lastDown = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            if (gamepad1.dpad_left) parkEnabled = false;
            if (gamepad1.dpad_right) parkEnabled = true;

            // Only count a press once, not every loop it is held
            if (gamepad1.dpad_up && !lastUp) {
                startDelaySeconds = Math.min(MAX_START_DELAY_SECONDS, startDelaySeconds + 1);
            }
            if (gamepad1.dpad_down && !lastDown) {
                startDelaySeconds = Math.max(0, startDelaySeconds - 1);
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addLine("Select with gamepad1, then press START");
            telemetry.addData("Park zone", parkFar ? "FAR (diagonal)" : "NEAR");
            telemetry.addData("Alliance (X=BLUE / B=RED)", isBlue ? "BLUE" : "RED");
            telemetry.addData("Start delay (dpad up/down)", "%d s", startDelaySeconds);
            telemetry.addData("Park (dpad left/right)", parkEnabled ? "ON" : "OFF");
            telemetry.update();
        }
    }

    private void launchPreloads() {
        // Spin up, then shoot once at speed, or give up after the timeout and shoot anyway
        launcherMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < LAUNCHER_SPINUP_TIMEOUT_SECONDS) {
            if (Math.abs(launcherMotor.getVelocity() - LAUNCHER_TARGET_VELOCITY) <= LAUNCHER_READY_TOLERANCE) {
                break;
            }
            telemetry.addData("Step", "Spinning up launcher");
            telemetry.addData("Launcher", "%.0f / %.0f ticks/s", launcherMotor.getVelocity(), LAUNCHER_TARGET_VELOCITY);
            telemetry.update();
        }

        feederServo.setPower(BiobuzzTeleOp.FEEDER_POWER);
        if (BiobuzzTeleOp.RUN_INTAKE_WHILE_FEEDING) {
            intakeMotor.setPower(BiobuzzTeleOp.INTAKE_POWER);
        }
        timer.reset();
        while (opModeIsActive() && timer.seconds() < FEED_SECONDS) {
            telemetry.addData("Step", "Launching pre-loads");
            telemetry.addData("Launcher", "%.0f / %.0f ticks/s", launcherMotor.getVelocity(), LAUNCHER_TARGET_VELOCITY);
            telemetry.update();
        }

        feederServo.setPower(0);
        intakeMotor.setPower(0);
        launcherMotor.setPower(0);
    }

    /** Drives straight toward the launcher end of the robot (negative inches = backwards). */
    private void driveInches(double inches) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH * DRIVE_DIRECTION);
        moveTo(ticks, ticks, DRIVE_POWER, "Drive " + inches + " in");
    }

    /** Turns in place. Angle is for BLUE; mirrored on RED. */
    private void turnDegrees(double blueDegrees) {
        double degrees = isBlue ? blueDegrees : -blueDegrees;
        double wheelInches = Math.PI * TRACK_WIDTH_INCHES * degrees / 360.0;
        int ticks = (int) Math.round(wheelInches * TICKS_PER_INCH);
        moveTo(ticks, -ticks, TURN_POWER, "Turn " + degrees + " deg");
    }

    private void moveTo(int leftTicks, int rightTicks, double power, String step) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + leftTicks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + rightTicks);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));

        timer.reset();
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())
                && timer.seconds() < MOVE_TIMEOUT_SECONDS) {
            telemetry.addData("Step", step);
            telemetry.addData("Left", "%d / %d", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Right", "%d / %d", rightDrive.getCurrentPosition(), rightDrive.getTargetPosition());
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void waitSeconds(double seconds) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            idle();
        }
    }

    private void stopAll() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intakeMotor.setPower(0);
        launcherMotor.setPower(0);
        feederServo.setPower(0);
    }
}
