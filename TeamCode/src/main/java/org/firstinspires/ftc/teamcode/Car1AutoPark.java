package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Backup auto: only LEAVE + PARK (8 points). Works for red and blue (field is point-symmetric about its center).
 * Start: back of robot against the wall at your own driver area, front of robot facing into the field, parked next to
 * the LOADING ZONE (the LOADING ZONE is on the robot's left; the robot must not touch the LOADING ZONE tape).
 *   1 forward 200 away from the wall (LEAVE)
 *   2 550 to the robot's left, so part of the robot is in the LOADING ZONE (PARK)
 */
@Autonomous(name = "Car1 Auto Park Only", group = "Car1", preselectTeleOp = "Car1 TeleOp")
public class Car1AutoPark extends Car1AutoV2Base {
    @Override protected double[][] path() {
        return new double[][]{{200, 0, 0}, {200, 550, 0}};
    }
    @Override protected boolean shoots() { return false; }
    @Override protected String title() { return "Car1 Auto Park Only (back of robot against your driver-area wall, LOADING ZONE on the robot's left)"; }
}
