package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BIOBUZZ (2026-2027) two-driver TeleOp for the REV DUO FTC Starter Bot.
 * Same robot and settings as BiobuzzTeleOp, but the controls are split across two gamepads.
 *
 * Robot configuration names (match REV's starter bot config):
 *   Motors: leftDrive (port 0), rightDrive (port 1), launcherMotor (port 2, LAUNCHES POLLEN),
 *           intakeMotor (port 3, INTAKE - pulls POLLEN in)
 *   Servos: feederServo (port 0, continuous feeder), pokerServo (port 1, pollen poker)
 *   Camera: Webcam 1 (USB), facing the same way as the launcher
 *
 * Controls (gamepad1 - DRIVER):
 *   Left stick Y    - drive forward / backward
 *   Right stick X   - turn left / right
 *   Left trigger    - hold for slow, precise driving
 *   Right trigger   - hold to AUTO-AIM at the HIVE with Webcam 1 (forward/back stays on the left stick)
 *
 * Controls (gamepad2 - OPERATOR):
 *   Left bumper     - launch: spin up launcher, then feed once it is at speed
 *   Right bumper    - press to start intake, press again to stop
 *   Y               - press to start launcher, press again to stop
 *   D-pad up/down   - run feeder forward/reverse manually (hold)
 *   D-pad left/right- trim launcher speed down/up if shots land short/long
 *   X               - press to start feeder (feederServo), press again to stop
 *   A / B           - pollen poker down / up
 *
 * While the driver auto-aims, launching waits until the robot is lined up. Both gamepads rumble when READY.
 * Pick "BIOBUZZ TeleOp 2 Gamepads BLUE" or "... RED" to match your alliance (TwoGamepadTeleOpBlue / Red).
 * The alliance decides which HIVE auto-aim looks at.
 * Whenever Webcam 1 sees the HIVE, launcher speed is set from the distance (see HiveAimer).
 */
public abstract class TWO_GamepadOpMode extends LinearOpMode {

        // Launcher target in encoder ticks per second, used when the camera can't see the HIVE.
    // Tune on the field: raise it if shots fall short of the CELL.
    static final double LAUNCHER_TARGET_VELOCITY = 2000;
    // Feeder only runs once the launcher is within this many ticks/sec of the target
    static final double LAUNCHER_READY_TOLERANCE = 100;

    static final double INTAKE_POWER = 0.70;
    static final double FEEDER_POWER = 1.0;
    // Also run the intake while feeding, to push POLLEN up into the feeder
    static final boolean RUN_INTAKE_WHILE_FEEDING = true;

    // REV Smart Robot Servo turns 270 degrees over a 500-2500 us pulse. The SDK defaults to 600-2400 us,
    // so the poker widens it in init to make these angles real degrees.
    static final double SERVO_RANGE_DEGREES = 270;
    // Poker angles, in degrees from one end of the servo travel. Tune these on the robot:
    //   leaning back -> lower POKER_UP_DEGREES, leaning forward -> raise it.
    static final double POKER_UP_DEGREES   = 270;  // upright (measured with Poker Servo Test: Triangle = 1.0 = up)
    static final double POKER_DOWN_DEGREES = 0;    // as far down as the servo can go
    static final double POKER_UP   = POKER_UP_DEGREES / SERVO_RANGE_DEGREES;
    static final double POKER_DOWN = POKER_DOWN_DEGREES / SERVO_RANGE_DEGREES;

    static final double DRIVE_MAX_SPEED  = 1.0;
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
    private HiveAimer aimer;

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

    private final boolean isBlue;
    private boolean autoAiming = false;
    private double launcherTarget = LAUNCHER_TARGET_VELOCITY;
    private double velocityTrim = 0;

    protected TWO_GamepadOpMode(boolean isBlue) {
        this.isBlue = isBlue;
    }

    @Override
    public void runOpMode() {
        leftDrive   = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class, "rightDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");
        pokerServo  = hardwareMap.get(ServoImplEx.class, "pokerServo");
        pokerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        aimer       = new HiveAimer(hardwareMap, isBlue);

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

        // While waiting for START, show whether the camera can see our HIVE
        while (!isStarted() && !isStopRequested()) {
            aimer.update();

            telemetry.addData("Status", "Initialized. Press START when TELEOP begins.");
            telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
            aimer.addTelemetry(telemetry);
            telemetry.update();
        }
        matchTimer.reset();

        while (opModeIsActive()) {
            aimer.update();
            drive();
            boolean launcherReady = runLauncher();
            runPoker();
            rumbleWhenReady(launcherReady);
            showTelemetry(launcherReady);
        }

        stopAll();
    }

    /** Driver (gamepad1) only drives the robot, and can hand turning over to auto-aim. */
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

    /** Operator (gamepad2) runs launcher, feeder and intake. Returns true when the launcher is at launch speed. */
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
            launcherOn = !launcherOn;
        }
        if (gamepad2.dpad_right && !lastDpadRight) {
            velocityTrim += VELOCITY_TRIM_STEP;
        }
        if (gamepad2.dpad_left && !lastDpadLeft) {
            velocityTrim -= VELOCITY_TRIM_STEP;
        }
        lastRightBumper = gamepad2.right_bumper;
        lastX = gamepad2.x;
        lastY = gamepad2.y;
        lastDpadRight = gamepad2.dpad_right;
        lastDpadLeft = gamepad2.dpad_left;

        // ---------------- Launcher ----------------
        // Speed comes from the distance to the HIVE when the camera sees it, plus the operator's trim
        launcherTarget = aimer.getLauncherVelocity(LAUNCHER_TARGET_VELOCITY) + velocityTrim;
        // Only ever spins in the shooting direction
        if (launching || launcherOn) {
            launcherMotor.setVelocity(launcherTarget);
        } else {
            launcherMotor.setPower(0);
        }
        boolean launcherReady =
                Math.abs(launcherMotor.getVelocity() - launcherTarget) < LAUNCHER_READY_TOLERANCE;

        // ---------------- Feeder ----------------
        // While the driver auto-aims, hold the shot until the robot is lined up so POLLEN isn't launched mid-turn
        boolean feeding = launching && launcherReady && (!autoAiming || aimer.isAimed());
        // Feeder is controlled separately from the intake. D-pad overrides the X toggle while held.
        if (gamepad2.dpad_down) {
            feederServo.setPower(-FEEDER_POWER);
        } else if (gamepad2.dpad_up || feederOn || feeding) {
            feederServo.setPower(FEEDER_POWER);
        } else {
            feederServo.setPower(0);
        }

        // ---------------- Intake ----------------
        // Only ever spins in the direction that pulls POLLEN in
        if (intakeOn || (feeding && RUN_INTAKE_WHILE_FEEDING)) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0);
        }

        return launcherReady;
    }

    private void runPoker() {
        if (gamepad2.a) {
            pokerServo.setPosition(POKER_DOWN);
        } else if (gamepad2.b) {
            pokerServo.setPosition(POKER_UP);
        }
    }

    /** Short rumble on both gamepads the moment the robot is aimed at the HIVE and the launcher is at speed. */
    private void rumbleWhenReady(boolean launcherReady) {
        boolean shotReady = aimer.isAimed() && launcherReady && launcherMotor.getVelocity() > 0;
        if (shotReady && !lastShotReady) {
            gamepad1.rumble(READY_RUMBLE_MS);
            gamepad2.rumble(READY_RUMBLE_MS);
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
        aimer.addTelemetry(telemetry);
        telemetry.addData("Auto-aim", autoAiming ? (aimer.isAimed() ? "LOCKED" : "TURNING") : "off");
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
        aimer.close();
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
