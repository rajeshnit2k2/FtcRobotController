package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BIOBUZZ Auto that parks in the closest LOADING ZONE. All the AUTO code is in BiobuzzAuto. */
@Autonomous(name = "BIOBUZZ Auto NEAR", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class BiobuzzAutoNear extends BiobuzzAuto {
    public BiobuzzAutoNear() {
        super(false);
    }
}
