package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
 *   Motors: leftDrive (port 0), rightDrive (port 1), intakeMotor (port 2, LAUNCHES - used as the flywheel),
 *           flywheel (port 3, INTAKE - pulls POLLEN in)
 *   Servos: crServo (port 0, continuous feeder), pollenServo (port 1, pollen poker)
 *   Camera: Webcam 1 (USB), facing the same way as the launcher
 *
 * Controls (gamepad1):
 *   Left stick Y    - drive forward / backward
 *   Right stick X   - turn left / right
 *   Left bumper     - launch: spin up flywheel, then feed once it is at speed
 *   Right bumper    - press to start intake, press again to stop
 *   Y               - press to start flywheel, press again to stop
 *   D-pad up/down   - run feeder forward/reverse manually (hold)
 *   D-pad left/right- trim flywheel speed down/up if shots land short/long
 *   X               - press to start feeder (crServo), press again to stop
 *   A / B           - pollen poker down / up
 *   Left trigger    - hold for slow, precise driving
 *   Right trigger   - hold to AUTO-AIM at the HIVE with Webcam 1 (forward/back stays on the left stick).
 *                     While held, launching waits until the robot is lined up. Gamepad rumbles when READY.
 *
 * During INIT: X = BLUE alliance, B = RED alliance (defaults to what AUTO used). Picks which HIVE to aim at.
 * Whenever Webcam 1 sees the HIVE, flywheel speed is set from the distance (see HiveAimer).
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BiobuzzTeleOp extends LinearOpMode {

    // Flywheel target in encoder ticks per second, used when the camera can't see the HIVE.
    // Tune on the field: raise it if shots fall short of the CELL.
    static final double FLYWHEEL_TARGET_VELOCITY = 2000;
    // Feeder only runs once the flywheel is within this many ticks/sec of the target
    static final double FLYWHEEL_READY_TOLERANCE = 100;

    static final double INTAKE_POWER = 0.67;
    static final double FEEDER_POWER = 1.0;
    // Also run the intake while feeding, to push POLLEN up into the feeder
    static final boolean RUN_INTAKE_WHILE_FEEDING = true;

    // REV Smart Robot Servo turns 270 degrees over a 500-2500 us pulse. The SDK defaults to 600-2400 us,
    // so the poker widens it in init to make these angles real degrees.
    static final double SERVO_RANGE_DEGREES = 270;
    // Poker angles, in degrees from one end of the servo travel. Tune these on the robot:
    //   leaning back -> lower POKER_UP_DEGREES, leaning forward -> raise it.
    static final double POKER_UP_DEGREES   = 80;   // upright (was 95, which leaned about 15 degrees back)
    static final double POKER_DOWN_DEGREES = 0;    // as far down as the servo can go
    static final double POKER_UP   = POKER_UP_DEGREES / SERVO_RANGE_DEGREES;
    static final double POKER_DOWN = POKER_DOWN_DEGREES / SERVO_RANGE_DEGREES;

    static final double DRIVE_MAX_SPEED  = 1.0;
    static final double DRIVE_SLOW_SPEED = 0.35;
    static final double STICK_DEADZONE   = 0.5;

    // Each D-pad left/right press changes the flywheel speed by this many ticks/sec
    static final double VELOCITY_TRIM_STEP = 25;
    static final int READY_RUMBLE_MS = 200;

    static final double TELEOP_SECONDS = 120;
    static final double FLOWER_NECTAR_ALLOWED_SECONDS_LEFT = 60;
    static final double PARK_WARNING_SECONDS_LEFT = 20;

    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private DcMotor   intakeMotor;
    private DcMotorEx flywheel;
    private CRServo   feeder;
    private ServoImplEx poker;
    private HiveAimer aimer;

    private final ElapsedTime matchTimer = new ElapsedTime();

    // Toggle state: each press of the button flips the mechanism on/off
    private boolean intakeOn = false;
    private boolean feederOn = false;
    private boolean flywheelOn = false;
    private boolean lastRightBumper = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastShotReady = false;

    private boolean isBlue = HiveAimer.lastAllianceBlue;
    private boolean autoAiming = false;
    private double flywheelTarget = FLYWHEEL_TARGET_VELOCITY;
    private double velocityTrim = 0;

    @Override
    public void runOpMode() {
        leftDrive   = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class, "rightDrive");
        // Config names are swapped on this robot: port 2 "intakeMotor" throws POLLEN, port 3 "flywheel" pulls it in
        intakeMotor = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel    = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        feeder      = hardwareMap.get(CRServo.class, "crServo");
        poker       = hardwareMap.get(ServoImplEx.class, "pollenServo");
        poker.setPwmRange(new PwmControl.PwmRange(500, 2500));
        aimer       = new HiveAimer(hardwareMap);

        // Flip any of these if that mechanism runs backwards
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Velocity control keeps every shot the same speed as the battery drains
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stand the poker upright as soon as the OpMode is INITed. Nothing else moves during INIT (G403).
        poker.setPosition(POKER_UP);

        // Pick the alliance while waiting for START, and show whether the camera can see that HIVE
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            aimer.setAlliance(isBlue);
            aimer.update();

            telemetry.addData("Status", "Initialized. Press START when TELEOP begins.");
            telemetry.addData("Alliance (X blue / B red)", isBlue ? "BLUE" : "RED");
            aimer.addTelemetry(telemetry);
            telemetry.update();
        }
        HiveAimer.lastAllianceBlue = isBlue;
        // X was used to pick the alliance; don't let that same press toggle the feeder
        lastX = gamepad1.x;
        matchTimer.reset();

        while (opModeIsActive()) {
            aimer.update();
            drive();
            boolean flywheelReady = runLauncher();
            runPoker();
            rumbleWhenReady(flywheelReady);
            showTelemetry(flywheelReady);
        }

        stopAll();
    }

    private void drive() {
        double forward = applyDeadzone(-gamepad1.left_stick_y);
        double turn    = applyDeadzone(gamepad1.right_stick_x);

        // Auto-aim takes over turning only while the HIVE is in view; otherwise the driver keeps control
        autoAiming = gamepad1.right_trigger > 0.5 && aimer.hasTarget();
        if (autoAiming) {
            turn = aimer.getTurnPower();
        }

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
        boolean launching = gamepad1.left_bumper;

        // Only flip on the moment the button goes down, not every loop while it is held
        if (gamepad1.right_bumper && !lastRightBumper) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.x && !lastX) {
            feederOn = !feederOn;
        }
        if (gamepad1.y && !lastY) {
            flywheelOn = !flywheelOn;
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            velocityTrim += VELOCITY_TRIM_STEP;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            velocityTrim -= VELOCITY_TRIM_STEP;
        }
        lastRightBumper = gamepad1.right_bumper;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;

        // ---------------- Flywheel ----------------
        // Speed comes from the distance to the HIVE when the camera sees it, plus the driver's trim
        flywheelTarget = aimer.getFlywheelVelocity(FLYWHEEL_TARGET_VELOCITY) + velocityTrim;
        // Only ever spins in the shooting direction
        if (launching || flywheelOn) {
            flywheel.setVelocity(flywheelTarget);
        } else {
            flywheel.setPower(0);
        }
        boolean flywheelReady =
                Math.abs(flywheel.getVelocity() - flywheelTarget) < FLYWHEEL_READY_TOLERANCE;

        // ---------------- Feeder ----------------
        // While auto-aiming, hold the shot until the robot is lined up so POLLEN isn't launched mid-turn
        boolean feeding = launching && flywheelReady && (!autoAiming || aimer.isAimed());
        // Feeder is controlled separately from the intake. D-pad overrides the X toggle while held.
        if (gamepad1.dpad_down) {
            feeder.setPower(-FEEDER_POWER);
        } else if (gamepad1.dpad_up || feederOn || feeding) {
            feeder.setPower(FEEDER_POWER);
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
        if (gamepad1.a) {
            poker.setPosition(POKER_DOWN);
        } else if (gamepad1.b) {
            poker.setPosition(POKER_UP);
        }
    }

    /** Short rumble the moment the robot is aimed at the HIVE and the flywheel is at speed. */
    private void rumbleWhenReady(boolean flywheelReady) {
        boolean shotReady = aimer.isAimed() && flywheelReady && flywheel.getVelocity() > 0;
        if (shotReady && !lastShotReady) {
            gamepad1.rumble(READY_RUMBLE_MS);
        }
        lastShotReady = shotReady;
    }

    private void showTelemetry(boolean flywheelReady) {
        double secondsLeft = Math.max(0, TELEOP_SECONDS - matchTimer.seconds());

        telemetry.addData("Time Left", "%d:%02d", (int) secondsLeft / 60, (int) secondsLeft % 60);
        if (secondsLeft <= PARK_WARNING_SECONDS_LEFT) {
            telemetry.addLine(">>> GO PARK IN THE LOADING ZONE <<<");
        } else if (secondsLeft <= FLOWER_NECTAR_ALLOWED_SECONDS_LEFT) {
            telemetry.addLine(">>> NECTAR into FLOWERS now allowed <<<");
        }
        aimer.addTelemetry(telemetry);
        telemetry.addData("Auto-aim", autoAiming ? (aimer.isAimed() ? "LOCKED" : "TURNING") : "off");
        telemetry.addData("Flywheel", "%.0f / %.0f ticks/s %s %s",
                flywheel.getVelocity(), flywheelTarget, flywheelReady ? "READY" : "",
                flywheelOn ? "(ON)" : "");
        telemetry.addData("Velocity trim", "%+.0f ticks/s", velocityTrim);
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
        aimer.close();
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
// done
