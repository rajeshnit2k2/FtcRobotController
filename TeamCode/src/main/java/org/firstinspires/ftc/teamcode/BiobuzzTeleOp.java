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
 *   Motors: leftDrive (port 0), rightDrive (port 1), launcherMotor (port 2, LAUNCHES POLLEN),
 *           intakeMotor (port 3, INTAKE - pulls POLLEN in)
 *   Servos: feederServo (port 0, continuous feeder), pokerServo (port 1, pollen poker)
 *
 * Controls (gamepad1):
 *   Left stick Y    - drive forward / backward
 *   Right stick X   - turn left / right
 *   Left bumper     - launch: spin up launcher, then feed once it is at speed
 *   Right bumper    - press to start intake, press again to stop
 *   Y               - ONE-BUTTON LAUNCH: press to spin up the launcher; the feeder starts by itself once
 *                     the launcher reaches full speed. Press again to stop both.
 *   D-pad up/down   - run feeder forward/reverse manually (hold)
 *   D-pad left/right- trim launcher speed down/up if shots land short/long
 *   X (Square)      - DISABLED so it can't be pressed by mistake (set SQUARE_FEEDER_TOGGLE_ENABLED = true
 *                     to make it toggle the feeder again)
 *   A / B           - pollen poker down / up
 *   Left trigger    - hold for slow, precise driving
 *
 * The gamepad rumbles when the launcher reaches full speed.
 * One program for both alliances (no camera / AprilTags needed).
 * Launcher speed is always LAUNCHER_POWER (plus D-pad trim).
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BiobuzzTeleOp extends LinearOpMode {

    // Launcher speed as a fraction of full speed (0.0 - 1.0), like INTAKE_POWER.
    // Tune on the field: raise it if shots fall short of the CELL.
    static final double LAUNCHER_POWER = 0.62;
    // Launcher motor's full speed in encoder ticks/sec. REV HD Hex Motor (no gearbox): 6000 RPM x 28 ticks = 2800.
    // Change this if the launcher uses a different motor or gearbox.
    static final double LAUNCHER_MAX_VELOCITY = 2000;
    // Velocity control still holds this speed steady as the battery drains
    static final double LAUNCHER_TARGET_VELOCITY = LAUNCHER_POWER * LAUNCHER_MAX_VELOCITY;
    // Feeder only runs once the launcher is within this many ticks/sec of the target
    static final double LAUNCHER_READY_TOLERANCE = 100;

    static final double INTAKE_POWER = 1.0;
    static final double FEEDER_POWER = 1.0;
    // Also run the intake while feeding, to push POLLEN up into the feeder
    static final boolean RUN_INTAKE_WHILE_FEEDING = true;
    // Square (X) used to toggle the feeder; it's off so a stray press can't start the feeder
    static final boolean SQUARE_FEEDER_TOGGLE_ENABLED = false;

    // REV Smart Robot Servo turns 270 degrees over a 500-2500 us pulse. The SDK defaults to 600-2400 us,
    // so the poker widens it in init to make these angles real degrees.
    static final double SERVO_RANGE_DEGREES = 270;
    // Poker angles, in degrees from one end of the servo travel. Tune these on the robot:
    //   leaning back -> lower POKER_UP_DEGREES, leaning forward -> raise it.
    static final double POKER_UP_DEGREES   = 270;  // upright (measured with Poker Servo Test: Triangle = 1.0 = up)
    static final double POKER_DOWN_DEGREES = 0;    // as far down as the servo can go
    static final double POKER_UP   = POKER_UP_DEGREES / SERVO_RANGE_DEGREES;
    static final double POKER_DOWN = POKER_DOWN_DEGREES / SERVO_RANGE_DEGREES;

    static final double DRIVE_MAX_SPEED  = 0.70;
    static final double DRIVE_SLOW_SPEED = 0.35;
    static final double STICK_DEADZONE   = 0.5;

    // Each D-pad left/right press changes the launcher speed by this many ticks/sec
    static final double VELOCITY_TRIM_STEP = 25;
    static final int READY_RUMBLE_MS = 200;

    static final double TELEOP_SECONDS = 120;
    static final double FLOWER_NECTAR_ALLOWED_SECONDS_LEFT = 60;
    static final double PARK_WARNING_SECONDS_LEFT = 20;

    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private DcMotor   intakeMotor;
    private DcMotorEx launcherMotor;
    private CRServo   feederServo;
    private ServoImplEx pokerServo;

    private final ElapsedTime matchTimer = new ElapsedTime();

    // Toggle state: each press of the button flips the mechanism on/off
    private boolean intakeOn = false;
    private boolean feederOn = false;
    private boolean launcherOn = false;
    private boolean lastRightBumper = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastShotReady = false;

    private double launcherTarget = LAUNCHER_TARGET_VELOCITY;
    private double velocityTrim = 0;

    @Override
    public void runOpMode() {
        leftDrive   = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class, "rightDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");
        pokerServo  = hardwareMap.get(ServoImplEx.class, "pokerServo");
        pokerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Flip any of these if that mechanism runs backwards
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);
        feederServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Velocity control keeps every shot the same speed as the battery drains
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stand the poker upright as soon as the OpMode is INITed. Nothing else moves during INIT (G403).
        pokerServo.setPosition(POKER_UP);

        telemetry.addData("Status", "Initialized. Press START when TELEOP begins.");
        telemetry.update();
        waitForStart();
        matchTimer.reset();

        while (opModeIsActive()) {
            drive();
            boolean launcherReady = runLauncher();
            runPoker();
            rumbleWhenReady(launcherReady);
            showTelemetry(launcherReady);
        }

        stopAll();
    }

    private void drive() {
        // Left stick is reversed so pushing it forward drives toward the launcher end of the robot.
        // (Stick Y is negative when pushed forward.) Remove the reversal by putting the minus sign back.
        double forward = applyDeadzone(gamepad1.left_stick_y);
        double turn    = applyDeadzone(gamepad1.right_stick_x);

        double left  = forward + turn;
        double right = forward - turn;

        // Normalize so turning while driving never asks for more than full power
        double max = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        double speed = gamepad1.left_trigger > 0.5 ? DRIVE_SLOW_SPEED : DRIVE_MAX_SPEED;

        leftDrive.setPower(left / max * speed);
        rightDrive.setPower(right / max * speed);
    }

    /** Runs launcher, feeder and intake. Returns true when the launcher is at launch speed. */
    private boolean runLauncher() {
        // Left bumper (hold) or the Y toggle both mean: spin up, then feed once at speed
        boolean launching = gamepad1.left_bumper || launcherOn;

        // Only flip on the moment the button goes down, not every loop while it is held
        if (gamepad1.right_bumper && !lastRightBumper) {
            intakeOn = !intakeOn;
        }
        if (SQUARE_FEEDER_TOGGLE_ENABLED && gamepad1.x && !lastX) {
            feederOn = !feederOn;
        }
        if (gamepad1.y && !lastY) {
            launcherOn = !launcherOn;
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

        // ---------------- Launcher ----------------
        // Fixed speed (LAUNCHER_POWER) plus the driver's D-pad trim
        launcherTarget = LAUNCHER_TARGET_VELOCITY + velocityTrim;
        // Only ever spins in the shooting direction
        if (launching) {
            launcherMotor.setVelocity(launcherTarget);
        } else {
            launcherMotor.setPower(0);
        }
        boolean launcherReady =
                Math.abs(launcherMotor.getVelocity() - launcherTarget) < LAUNCHER_READY_TOLERANCE;

        // ---------------- Feeder ----------------
        // Feed only once the launcher is at full speed
        boolean feeding = launching && launcherReady;
        // Feeder is controlled separately from the intake. D-pad overrides the X toggle while held.
        if (gamepad1.dpad_down) {
            feederServo.setPower(-FEEDER_POWER);
        } else if (gamepad1.dpad_up || feederOn || feeding) {
            feederServo.setPower(FEEDER_POWER);
        } else {
            feederServo.setPower(0);
        }

        // ---------------- Intake ----------------
        // Only ever spins in the direction that pulls POLLEN in.
        // Triangle (Y) runs only the launcher and feeder; the intake helps feed only on a left-bumper launch.
        if (intakeOn || (feeding && gamepad1.left_bumper && RUN_INTAKE_WHILE_FEEDING)) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0);
        }

        return launcherReady;
    }

    private void runPoker() {
        if (gamepad1.a) {
            pokerServo.setPosition(POKER_DOWN);
        } else if (gamepad1.b) {
            pokerServo.setPosition(POKER_UP);
        }
    }

    /** Short rumble the moment the launcher reaches full speed. */
    private void rumbleWhenReady(boolean launcherReady) {
        boolean shotReady = launcherReady && launcherMotor.getVelocity() > 0;
        if (shotReady && !lastShotReady) {
            gamepad1.rumble(READY_RUMBLE_MS);
        }
        lastShotReady = shotReady;
    }

    private void showTelemetry(boolean launcherReady) {
        double secondsLeft = Math.max(0, TELEOP_SECONDS - matchTimer.seconds());

        telemetry.addData("Time Left", "%d:%02d", (int) secondsLeft / 60, (int) secondsLeft % 60);
        if (secondsLeft <= PARK_WARNING_SECONDS_LEFT) {
            telemetry.addLine(">>> GO PARK IN THE LOADING ZONE <<<");
        } else if (secondsLeft <= FLOWER_NECTAR_ALLOWED_SECONDS_LEFT) {
            telemetry.addLine(">>> NECTAR into FLOWERS now allowed <<<");
        }
        telemetry.addData("Launcher", "%.0f / %.0f ticks/s %s %s",
                launcherMotor.getVelocity(), launcherTarget, launcherReady ? "READY" : "",
                launcherOn ? "(ON)" : "");
        telemetry.addData("Velocity trim", "%+.0f ticks/s", velocityTrim);
        telemetry.addData("Drive", "L %.2f  R %.2f", leftDrive.getPower(), rightDrive.getPower());
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Feeder", "%.2f %s", feederServo.getPower(), feederOn ? "(ON)" : "");
        telemetry.addData("Poker", "%.2f", pokerServo.getPosition());
        telemetry.update();
    }

    private void stopAll() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intakeMotor.setPower(0);
        launcherMotor.setPower(0);
        feederServo.setPower(0);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
