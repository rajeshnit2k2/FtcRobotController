package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BIOBUZZ (2026-2027) TeleOp for the REV DUO FTC Starter Bot.
 *
 * Game reminders (Competition Manual TU03):
 *   - TELEOP is 2:00. LAUNCH POLLEN/NECTAR into your upward-facing CELL to TIP the HIVE (20 pts each).
 *   - NECTAR may only go into a FLOWER in the last 60 seconds (G410).
 *   - Never CONTROL more than 4 SCORING ELEMENTS at a time (G407).
 *   - End the match at least partially in your LOADING ZONE for PARK (5 pts), and stop moving at 0:00 (G404).
 *
 * Robot configuration names (match REV's starter bot config):
 *   Motors: leftDrive (port 0), rightDrive (port 1), intakeMotor (port 2), flywheel (port 3)
 *   Servos: crServo (port 0, continuous feeder), pollenServo (port 1, pollen poker)
 *
 * Controls - gamepad1 (driver):
 *   Left stick      - arcade drive (Y = forward/back, X = turn)
 *   Left trigger    - hold for slow, precise driving
 *
 * Controls - gamepad2 (operator):
 *   Left bumper     - launch: spin up flywheel, then feed once it is at speed
 *   Right bumper    - press to start intake + feeder, press again to stop both
 *   Y               - press to start flywheel, press again to stop
 *   D-pad up/down   - run feeder forward/reverse manually (hold)
 *   X               - press to start feeder (crServo), press again to stop
 *   A / B           - pollen poker down / up
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BiobuzzTeleOp extends LinearOpMode {

    // Flywheel target in encoder ticks per second. Tune on the field: raise it if shots fall short of the CELL.
    static final double FLYWHEEL_TARGET_VELOCITY = 2000;
    // Feeder only runs once the flywheel is within this many ticks/sec of the target
    static final double FLYWHEEL_READY_TOLERANCE = 100;

    static final double INTAKE_POWER = 0.5;
    static final double FEEDER_POWER = 1.0;
    // Also run the intake while feeding, to push POLLEN up into the feeder
    static final boolean RUN_INTAKE_WHILE_FEEDING = true;

    // Pollen poker travel is capped at POKER_MAX_DEGREES. REV Smart Robot Servo default range is 270 degrees.
    static final double SERVO_RANGE_DEGREES = 270;
    static final double POKER_MAX_DEGREES   = 90;
    // Poker positions (0.0 to 1.0) within the capped range: 0.0 = 0 degrees, 1.0 = 90 degrees
    static final double POKER_DOWN = 0.0;
    static final double POKER_UP   = 0.90;

    static final double DRIVE_MAX_SPEED  = 1.0;
    static final double DRIVE_SLOW_SPEED = 0.35;
    static final double STICK_DEADZONE   = 0.5;

    static final double TELEOP_SECONDS = 120;
    static final double FLOWER_NECTAR_ALLOWED_SECONDS_LEFT = 60;
    static final double PARK_WARNING_SECONDS_LEFT = 20;

    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private DcMotor   intakeMotor;
    private DcMotorEx flywheel;
    private CRServo   feeder;
    private Servo     poker;

    private final ElapsedTime matchTimer = new ElapsedTime();

    // Toggle state: each press of the button flips the mechanism on/off
    private boolean intakeOn = false;
    private boolean feederOn = false;
    private boolean flywheelOn = false;
    private boolean lastRightBumper = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        leftDrive   = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class, "rightDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        flywheel    = hardwareMap.get(DcMotorEx.class, "flywheel");
        feeder      = hardwareMap.get(CRServo.class, "crServo");
        poker       = hardwareMap.get(Servo.class, "pollenServo");
        // Limit the poker so it can never turn past POKER_MAX_DEGREES
        poker.scaleRange(0.0, POKER_MAX_DEGREES / SERVO_RANGE_DEGREES);

        // Flip any of these if that mechanism runs backwards
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Velocity control keeps every shot the same speed as the battery drains
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Nothing moves during INIT, so it is safe to INIT during the AUTO-to-TELEOP transition (G403)
        telemetry.addData("Status", "Initialized. Press START when TELEOP begins.");
        telemetry.update();

        waitForStart();
        matchTimer.reset();
        poker.setPosition(POKER_UP);

        while (opModeIsActive()) {
            drive();
            boolean flywheelReady = runLauncher();
            runPoker();
            showTelemetry(flywheelReady);
        }

        stopAll();
    }

    private void drive() {
        double forward = applyDeadzone(-gamepad1.left_stick_y);
        double turn    = applyDeadzone(gamepad1.left_stick_x);

        double left  = forward + turn;
        double right = forward - turn;

        // Normalize so turning while driving never asks for more than full power
        double max = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        double speed = gamepad1.left_trigger > 0.5 ? DRIVE_SLOW_SPEED : DRIVE_MAX_SPEED;

        leftDrive.setPower(left / max * speed);
        rightDrive.setPower(right / max * speed);
    }

    /** Runs flywheel, feeder and intake. Returns true when the flywheel is at launch speed. */
    private boolean runLauncher() {
        boolean launching = gamepad2.left_bumper;

        // Only flip on the moment the button goes down, not every loop while it is held
        if (gamepad2.right_bumper && !lastRightBumper) {
            intakeOn = !intakeOn;
        }
        if (gamepad2.x && !lastX) {
            feederOn = !feederOn;
        }
        if (gamepad2.y && !lastY) {
            flywheelOn = !flywheelOn;
        }
        lastRightBumper = gamepad2.right_bumper;
        lastX = gamepad2.x;
        lastY = gamepad2.y;

        // ---------------- Flywheel ----------------
        // Only ever spins in the shooting direction
        if (launching || flywheelOn) {
            flywheel.setVelocity(FLYWHEEL_TARGET_VELOCITY);
        } else {
            flywheel.setPower(0);
        }
        boolean flywheelReady =
                Math.abs(flywheel.getVelocity() - FLYWHEEL_TARGET_VELOCITY) < FLYWHEEL_READY_TOLERANCE;

        // ---------------- Feeder ----------------
        boolean feeding = launching && flywheelReady;
        // Feeder also runs whenever the intake is switched on (right bumper)
        if (gamepad2.dpad_up || feederOn || intakeOn || feeding) {
            feeder.setPower(FEEDER_POWER);
        } else if (gamepad2.dpad_down) {
            feeder.setPower(-FEEDER_POWER);
        } else {
            feeder.setPower(0);
        }

        // ---------------- Intake ----------------
        // Only ever spins in the direction that pulls POLLEN in
        if (intakeOn || (feeding && RUN_INTAKE_WHILE_FEEDING)) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0);
        }

        return flywheelReady;
    }

    private void runPoker() {
        if (gamepad2.a) {
            poker.setPosition(POKER_DOWN);
        } else if (gamepad2.b) {
            poker.setPosition(POKER_UP);
        }
    }

    private void showTelemetry(boolean flywheelReady) {
        double secondsLeft = Math.max(0, TELEOP_SECONDS - matchTimer.seconds());

        telemetry.addData("Time Left", "%d:%02d", (int) secondsLeft / 60, (int) secondsLeft % 60);
        if (secondsLeft <= PARK_WARNING_SECONDS_LEFT) {
            telemetry.addLine(">>> GO PARK IN THE LOADING ZONE <<<");
        } else if (secondsLeft <= FLOWER_NECTAR_ALLOWED_SECONDS_LEFT) {
            telemetry.addLine(">>> NECTAR into FLOWERS now allowed <<<");
        }
        telemetry.addData("Flywheel", "%.0f / %.0f ticks/s %s %s",
                flywheel.getVelocity(), FLYWHEEL_TARGET_VELOCITY, flywheelReady ? "READY" : "",
                flywheelOn ? "(ON)" : "");
        telemetry.addData("Drive", "L %.2f  R %.2f", leftDrive.getPower(), rightDrive.getPower());
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Feeder", "%.2f %s", feeder.getPower(), feederOn ? "(ON)" : "");
        telemetry.addData("Poker", "%.2f", poker.getPosition());
        telemetry.update();
    }

    private void stopAll() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intakeMotor.setPower(0);
        flywheel.setPower(0);
        feeder.setPower(0);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
// done
