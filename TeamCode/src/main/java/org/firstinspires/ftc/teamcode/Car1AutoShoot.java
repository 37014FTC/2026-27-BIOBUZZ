package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Main auto: start with the front of the robot against the wall of the CELL that faces up in your alliance color,
 * lined up with the CELL -> shoot the preloaded POLLEN from where it stands -> LEAVE (back away from the wall)
 * -> drive along the side to the LOADING ZONE and PARK.
 * Works for red and blue (field is point-symmetric about its center). Start: front of robot against the wall,
 * robot in your own half, not touching the garden or flowers, not inside the LOADING ZONE.
 *
 * Path (Pinpoint coordinates, start is the origin, x toward the front of the robot +, y to the robot's left +, mm;
 * estimated from the field drawing, one tile = 609.6):
 *   1 back up 300 away from the wall (LEAVE)
 *   2 back up diagonally to x -560, 790 to the robot's right (to the column next to the HIVE stand, avoiding the HIVE)
 *   3 keep backing up to x -2450 (along this column to the row with the LOADING ZONE)
 *   4 move to the robot's right again to y -1130, so part of the robot is in the LOADING ZONE (PARK)
 */
@Autonomous(name = "Car1 Auto Shoot+Park", group = "Car1", preselectTeleOp = "Car1 TeleOp")
public class Car1AutoShoot extends Car1AutoV2Base {
    @Override protected double[][] path() {
        return new double[][]{{-300, 0, 0}, {-560, -790, 0}, {-2450, -790, 0}, {-2480, -1130, 0}};
    }
    @Override protected boolean shoots() { return true; }
    @Override protected String title() { return "Car1 Auto Shoot+Park (start: front of robot against the wall, lined up with the CELL)"; }
}
