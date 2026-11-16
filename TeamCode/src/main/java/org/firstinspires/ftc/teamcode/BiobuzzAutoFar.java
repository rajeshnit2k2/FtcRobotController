package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BIOBUZZ Auto that parks in the diagonally opposite (far) LOADING ZONE, using an L-shaped path around
 * the HIVE. All the AUTO code is in BiobuzzAuto.
 */
@Autonomous(name = "BIOBUZZ Auto FAR", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class BiobuzzAutoFar extends BiobuzzAuto {
    public BiobuzzAutoFar() {
        super(true);
    }
}
