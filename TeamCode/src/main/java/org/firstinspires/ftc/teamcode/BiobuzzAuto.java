package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
 *   2. Turn to face our HIVE using Webcam 1 while the flywheel spins up to the speed for that distance,
 *      then LAUNCH the 4 pre-loaded POLLEN into our upward-facing CELL (counts toward a HIVE TIP, 20 pts).
 *      If the camera can't see the HIVE, it launches straight ahead at the default speed like before.
 *   3. Drive off the wall for LEAVE (3 pts).
 *   4. (Optional) turn and drive into our LOADING ZONE for PARK (5 pts).
 *
 * Setup (G304): place the robot on your own half of the FIELD, touching the perimeter wall, outside the
 * LOADING ZONE, touching its 4 pre-loaded POLLEN, with the launcher aimed at your alliance's upward-facing CELL.
 *
 * During INIT, use gamepad1 to choose options (shown on the Driver Station):
 *   X = BLUE alliance, B = RED alliance
 *   D-pad up/down    = start delay +/- 1 second
 *   D-pad left/right = PARK leg off/on
 *
 * Driving here is time-based, so tune the *_SECONDS constants on a practice field for your start spot.
 */
@Autonomous(name = "BIOBUZZ Auto", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class BiobuzzAuto extends LinearOpMode {

    // Launcher settings, shared with TeleOp so both shoot the same
    static final double FLYWHEEL_TARGET_VELOCITY = BiobuzzTeleOp.FLYWHEEL_TARGET_VELOCITY;
    static final double FLYWHEEL_READY_TOLERANCE = BiobuzzTeleOp.FLYWHEEL_READY_TOLERANCE;
    // Also the longest we spend turning to face the HIVE before shooting anyway
    static final double FLYWHEEL_SPINUP_TIMEOUT_SECONDS = 3.0;
    // Long enough to push all 4 pre-loads through the launcher
    static final double FEED_SECONDS = 4.0;

    // LEAVE: drive straight away from the wall
    static final double LEAVE_POWER   = 0.5;
    static final double LEAVE_SECONDS = 0.8;

    // PARK: turn toward the LOADING ZONE, then drive into it. Mirrored automatically for RED vs BLUE.
    static final double TURN_POWER        = 0.4;
    static final double PARK_TURN_SECONDS  = 0.6;
    static final double PARK_DRIVE_SECONDS = 1.5;
    // +1 turns right on BLUE (left on RED). Flip to -1 if the robot turns the wrong way from your start spot.
    static final int BLUE_TURN_DIRECTION = 1;

    static final int MAX_START_DELAY_SECONDS = 15;

    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private DcMotor   intakeMotor;
    private DcMotorEx flywheel;
    private CRServo   feeder;
    private HiveAimer aimer;

    private boolean isBlue = true;
    private boolean parkEnabled = true;
    private int startDelaySeconds;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();
        selectOptions();

        // selectOptions() returns on START or STOP; bail out if it was STOP
        if (!opModeIsActive()) {
            aimer.close();
            return;
        }
        // TeleOp starts on the same alliance
        HiveAimer.lastAllianceBlue = isBlue;

        waitSeconds(startDelaySeconds);
        launchPreloads();
        drive(LEAVE_POWER, LEAVE_POWER, LEAVE_SECONDS);

        if (parkEnabled) {
            int turnDirection = isBlue ? BLUE_TURN_DIRECTION : -BLUE_TURN_DIRECTION;
            drive(TURN_POWER * turnDirection, -TURN_POWER * turnDirection, PARK_TURN_SECONDS);
            drive(LEAVE_POWER, LEAVE_POWER, PARK_DRIVE_SECONDS);
        }

        stopAll();
        telemetry.addData("Status", "AUTO complete");
        telemetry.update();
    }

    private void initHardware() {
        leftDrive   = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class, "rightDrive");
        // Config names are swapped on this robot: port 2 "intakeMotor" throws POLLEN, port 3 "flywheel" pulls it in
        intakeMotor = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel    = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        feeder      = hardwareMap.get(CRServo.class, "crServo");
        aimer       = new HiveAimer(hardwareMap);

        // Keep these in sync with BiobuzzTeleOp
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Lets the drive team pick alliance, delay and park while waiting for START. Robot stays still (G304.H). */
    private void selectOptions() {
        boolean lastUp = false, lastDown = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            if (gamepad1.dpad_left)  parkEnabled = false;
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

            // Lets the drive team check the camera can see the HIVE from the start spot
            aimer.setAlliance(isBlue);
            aimer.update();

            telemetry.addLine("Select with gamepad1, then press START");
            telemetry.addData("Alliance (X blue / B red)", isBlue ? "BLUE" : "RED");
            telemetry.addData("Start delay (dpad up/down)", "%d s", startDelaySeconds);
            telemetry.addData("Park (dpad left/right)", parkEnabled ? "ON" : "OFF");
            aimer.addTelemetry(telemetry);
            telemetry.update();
        }
    }

    private void launchPreloads() {
        // Spin up while turning to face the HIVE. Shoot once at speed AND aimed (or the HIVE isn't in view),
        // or give up after the timeout and shoot anyway.
        timer.reset();
        while (opModeIsActive() && timer.seconds() < FLYWHEEL_SPINUP_TIMEOUT_SECONDS) {
            double targetVelocity = aimAtHive();
            boolean atSpeed = Math.abs(flywheel.getVelocity() - targetVelocity) <= FLYWHEEL_READY_TOLERANCE;
            if (atSpeed && (aimer.isAimed() || !aimer.hasTarget())) {
                break;
            }
            telemetry.addData("Step", "Aiming and spinning up flywheel");
            aimer.addTelemetry(telemetry);
            telemetry.addData("Flywheel", "%.0f / %.0f ticks/s", flywheel.getVelocity(), targetVelocity);
            telemetry.update();
        }

        feeder.setPower(BiobuzzTeleOp.FEEDER_POWER);
        if (BiobuzzTeleOp.RUN_INTAKE_WHILE_FEEDING) {
            intakeMotor.setPower(BiobuzzTeleOp.INTAKE_POWER);
        }
        timer.reset();
        while (opModeIsActive() && timer.seconds() < FEED_SECONDS) {
            // Keep correcting aim and speed so later shots stay on target if the robot gets nudged
            double targetVelocity = aimAtHive();
            telemetry.addData("Step", "Launching pre-loads");
            aimer.addTelemetry(telemetry);
            telemetry.addData("Flywheel", "%.0f / %.0f ticks/s", flywheel.getVelocity(), targetVelocity);
            telemetry.update();
        }

        feeder.setPower(0);
        intakeMotor.setPower(0);
        flywheel.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /** One step of turning toward the HIVE and setting flywheel speed for its distance. Returns that speed. */
    private double aimAtHive() {
        aimer.update();
        double turn = aimer.getTurnPower();
        leftDrive.setPower(turn);
        rightDrive.setPower(-turn);

        double targetVelocity = aimer.getFlywheelVelocity(FLYWHEEL_TARGET_VELOCITY);
        flywheel.setVelocity(targetVelocity);
        return targetVelocity;
    }

    private void drive(double leftPower, double rightPower, double seconds) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        waitSeconds(seconds);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
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
        flywheel.setPower(0);
        feeder.setPower(0);
        aimer.close();
    }
}
