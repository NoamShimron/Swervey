// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DRIVE_GEAR_RATIO = 1.0 / 8.14;
    public static final double DRIVE_STEER_GEAR_RATIO = 7.0 / 150.0;
    public static final double DRIVE_WHEEL_RADIUS = 0.0508;
    public static final double DRIVE_OFFSET = 0.3675;

    public static final double TALONFX_CPR = 2048.0;
    public static final double CANCODER_RESOLUTION = 360.0 / 4096.0;

    public static final int FL_STR_ID = 1;
    public static final int FL_DR_ID = 2;
    public static final int FL_CAN_ID = 3;
    public static final int FR_STR_ID = 4;
    public static final int FR_DR_ID = 5;
    public static final int FR_CAN_ID = 6;
    public static final int BL_STR_ID = 7;
    public static final int BL_DR_ID = 8;
    public static final int BL_CAN_ID = 9;
    public static final int BR_STR_ID = 10;
    public static final int BR_DR_ID = 11;
    public static final int BR_CAN_ID = 12;


}
