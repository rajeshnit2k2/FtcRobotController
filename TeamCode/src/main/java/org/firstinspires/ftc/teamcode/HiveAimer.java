package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

/**
 * Finds our alliance's HIVE with "Webcam 1" and works out how to aim and how hard to launch at it.
 * Shared by BiobuzzTeleOp, TWO_GamepadOpMode and BiobuzzAuto.
 *
 * The SDK's BIOBUZZ tag library groups the 4 AprilTags on the bottom of each CELL into a "cluster", so the SDK
 * solves one steady pose from however many of the 4 tags it can see. Each HIVE has 2 CELLS:
 *   RED  HIVE: RED SCORING  = tags 30-33 (far side)    RED AUDIENCE  = tags 34-37 (audience side)
 *   BLUE HIVE: BLUE SCORING = tags 42-45 (far side)    BLUE AUDIENCE = tags 38-41 (audience side)
 *
 * The tags face DOWN at the floor, so only the upward-facing CELL's tags can be seen from the robot. Every
 * HIVE TIP swaps which CELL faces up, so we watch BOTH of our alliance's CELLS and aim at whichever one is
 * in view. That one is always the CELL we should be scoring in.
 *
 * Call update() once per loop, then read hasTarget(), getTurnPower(), isAimed() and getLauncherVelocity().
 *
 * Tuning order on the practice field:
 *   1. LAUNCHER_RIGHT_OF_CAMERA_INCHES - measure it on the robot.
 *   2. AIM_TRIM_DEGREES - if centered shots still land left/right of the CELL.
 *   3. TURN_GAIN / MIN_TURN_POWER - if auto-aim is sluggish (raise) or wobbles (lower).
 *   4. SHOT_RANGE_INCHES / SHOT_VELOCITY - shoot from each range and write down the velocity that scores.
 */
public class HiveAimer {

    static final String WEBCAM_NAME = "Webcam 1";
    // Cluster names from AprilTagGameDatabase.getBioBuzzTagLibrary()
    static final String[] RED_CELL_CLUSTERS  = { "RED SCORING",  "RED AUDIENCE" };
    static final String[] BLUE_CELL_CLUSTERS = { "BLUE SCORING", "BLUE AUDIENCE" };

    // How far the launcher sits to the RIGHT of the camera lens, looking from behind the robot (negative = left).
    // The aim is corrected for this so the launcher, not the camera, ends up pointing at the HIVE.
    // Measured: launcher center is 4" directly above/below the camera, so no left/right correction is needed.
    static final double LAUNCHER_RIGHT_OF_CAMERA_INCHES = 0.0;
    // Extra aim correction in degrees. Positive aims further LEFT. Use it if shots still land off to one side.
    static final double AIM_TRIM_DEGREES = 0.0;

    // Robot counts as aimed when within this many degrees of the HIVE
    static final double AIM_TOLERANCE_DEGREES = 2.0;
    // Turn power = heading error * TURN_GAIN, clipped to MAX_TURN_POWER
    static final double TURN_GAIN      = 0.015;
    // Smallest turn power that still moves the robot, so it does not stall just short of the target
    static final double MIN_TURN_POWER = 0.08;
    static final double MAX_TURN_POWER = 0.40;

    // Keep using the last sighting for this long if the tags flicker out of view for a moment
    static final double TARGET_HOLD_SECONDS = 0.3;
    // 0..1: how much each new range reading moves the smoothed range (lower = smoother, slower)
    static final double RANGE_SMOOTHING = 0.3;

    // Launcher velocity (ticks/s) that scores from each camera-to-HIVE range (inches). Must be in increasing range order.
    // STARTING GUESSES ONLY - shoot from each distance on the field and replace these with what actually scores.
    // Ranges between two points are blended; outside the table the nearest end is used.
    static final double[] SHOT_RANGE_INCHES = {   24,   36,   48,   60,   72,   96 };
    static final double[] SHOT_VELOCITY     = { 1700, 1850, 2000, 2150, 2300, 2600 };

    // Short exposure cuts motion blur so tags are still read while the robot turns. Raise gain if the image is too dark.
    static final int CAMERA_EXPOSURE_MS = 6;
    static final int CAMERA_GAIN        = 250;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final ElapsedTime sinceLastSeen = new ElapsedTime();
    private final boolean isBlue;
    private final String[] cellClusters;

    private boolean exposureSet = false;
    private boolean everSeen = false;
    // Which of our 2 CELLS we are currently aiming at
    private String currentCell = "";
    private int tagsSeenPercent;
    private double range;
    private double bearing;

    public HiveAimer(HardwareMap hardwareMap, boolean isBlue) {
        this.isBlue = isBlue;
        this.cellClusters = isBlue ? BLUE_CELL_CLUSTERS : RED_CELL_CLUSTERS;

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
        // 2 is a good trade-off between detection range and frame rate for 3.25" tags
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();
    }

    public boolean isBlue() {
        return isBlue;
    }

    /** Reads the latest camera detections. Call once per loop. */
    public void update() {
        setExposureOnceStreaming();

        // Find the best view of either of our CELLS. Normally only the upward-facing one is visible; if both
        // somehow are, prefer the one with more of its 4 tags found, then the closer one.
        AprilTagClusterDetection best = null;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (!(detection instanceof AprilTagClusterDetection) || detection.ftcPose == null) {
                continue;
            }
            AprilTagClusterDetection cell = (AprilTagClusterDetection) detection;
            if (!isOurCell(cell.metadata.shortName)) {
                continue;
            }
            if (best == null
                    || cell.percentClusterFound > best.percentClusterFound
                    || (cell.percentClusterFound == best.percentClusterFound && cell.ftcPose.range < best.ftcPose.range)) {
                best = cell;
            }
        }
        if (best == null) {
            return;
        }

        // After a HIVE TIP we switch to the other CELL; don't blend its range with the old CELL's
        boolean switchedCell = !best.metadata.shortName.equals(currentCell);
        currentCell = best.metadata.shortName;

        // Move the target from the camera's point of view to the launcher's.
        // ftcPose: x = inches right of the camera, y = inches straight ahead. Bearing is positive to the LEFT.
        double x = best.ftcPose.x - LAUNCHER_RIGHT_OF_CAMERA_INCHES;
        double y = best.ftcPose.y;
        bearing = -Math.toDegrees(Math.atan2(x, y)) + AIM_TRIM_DEGREES;
        range = (everSeen && !switchedCell) ? range + RANGE_SMOOTHING * (best.ftcPose.range - range)
                                            : best.ftcPose.range;
        tagsSeenPercent = best.percentClusterFound;
        everSeen = true;
        sinceLastSeen.reset();
    }

    private boolean isOurCell(String clusterName) {
        for (String name : cellClusters) {
            if (name.equals(clusterName)) {
                return true;
            }
        }
        return false;
    }

    public boolean hasTarget() {
        return everSeen && sinceLastSeen.seconds() < TARGET_HOLD_SECONDS;
    }

    public boolean isAimed() {
        return hasTarget() && Math.abs(bearing) <= AIM_TOLERANCE_DEGREES;
    }

    /** Inches from the camera to the HIVE tags (smoothed). Only meaningful when hasTarget(). */
    public double getRange() {
        return range;
    }

    /** Degrees from the launcher to the HIVE, positive = HIVE is to the LEFT. Only meaningful when hasTarget(). */
    public double getBearing() {
        return bearing;
    }

    /** Turn power that swings the robot to face the HIVE. Positive turns right, matching the drive code. */
    public double getTurnPower() {
        if (!hasTarget() || isAimed()) {
            return 0;
        }
        // Target to the left (positive bearing) means turn left (negative power)
        double turn = Range.clip(-bearing * TURN_GAIN, -MAX_TURN_POWER, MAX_TURN_POWER);
        if (Math.abs(turn) < MIN_TURN_POWER) {
            turn = Math.copySign(MIN_TURN_POWER, turn);
        }
        return turn;
    }

    /** Launcher velocity for the current range, or fallbackVelocity when the HIVE is not in view. */
    public double getLauncherVelocity(double fallbackVelocity) {
        if (!hasTarget()) {
            return fallbackVelocity;
        }
        int last = SHOT_RANGE_INCHES.length - 1;
        if (range <= SHOT_RANGE_INCHES[0]) {
            return SHOT_VELOCITY[0];
        }
        if (range >= SHOT_RANGE_INCHES[last]) {
            return SHOT_VELOCITY[last];
        }
        int i = 1;
        while (SHOT_RANGE_INCHES[i] < range) {
            i++;
        }
        double fraction = (range - SHOT_RANGE_INCHES[i - 1]) / (SHOT_RANGE_INCHES[i] - SHOT_RANGE_INCHES[i - 1]);
        return SHOT_VELOCITY[i - 1] + fraction * (SHOT_VELOCITY[i] - SHOT_VELOCITY[i - 1]);
    }

    public void addTelemetry(Telemetry telemetry) {
        if (hasTarget()) {
            telemetry.addData("HIVE", "%s  %.0f in  %+.1f deg  (%d%% of tags) %s",
                    currentCell, range, bearing, tagsSeenPercent, isAimed() ? "AIMED" : "");
        } else {
            telemetry.addData("HIVE", "%s not in view (camera %s)",
                    isBlue ? "BLUE" : "RED", visionPortal.getCameraState());
        }
    }

    public void close() {
        visionPortal.close();
    }

    /** Exposure can only be changed after the camera starts streaming, so keep trying each loop until it is. */
    private void setExposureOnceStreaming() {
        if (exposureSet || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(CAMERA_EXPOSURE_MS, TimeUnit.MILLISECONDS);
        visionPortal.getCameraControl(GainControl.class).setGain(CAMERA_GAIN);
        exposureSet = true;
    }
}
