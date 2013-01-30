/*
 * Defines the wiring of the 2012 robot
 */
package edu.wpi.first.wpilibj.templates;

public class Wiring {
    //
    // Jaguar IDs
    //
    static public final int DRIVE_LEFTREAR_JAGUAR_ID = 2;
    static public final int DRIVE_LEFTFRONT_JAGUAR_ID = 4;
    static public final int DRIVE_RIGHTREAR_JAGUAR_ID = 5;
    static public final int DRIVE_RIGHTFRONT_JAGUAR_ID = 3;

    static public final int TURRET_FLY_JAGUAR_ID = 10;
    static public final int TURRET_BELT_JAGUAR_ID = 7;
    static public final int TURRET_ROT_JAGUAR_ID = 6;
    //
    //Victors
    //
    static public final int ELEVATOR_PWM_CHANNEL = 1;
    static public final int ARM_PWM_CHANNEL = 3; //not certain
    static public final int ARM_PWM_ID = 4;
    //
    //SPIKE ID's
    //
    static public final int GATHER_FRONT_SPIKE_ID = 2;
    static public final int GATHER_BACK_SPIKE_ID = 3;
    
    //
    // Analog Input/Outputs
    //
    static public final int HORIZONTAL_GYRO_ANALOG_IN = 2;
    static public final int VERTICAL_GYRO_ANALOG_IN = 1;

    static public final int GATHER_FRONT_ANALOG_IN = 4;
    static public final int GATHER_BACK_ANALOG_IN = 3;

    static public final int ELEVATOR_BOT_ANALOG_IN = 5;
    static public final int ELEVATOR_MID_ANALOG_IN = 6;
    static public final int ELEVATOR_TOP_ANALOG_IN = 7;
    //
    //buttons
    //
    static public final int A_BUTTON = 1;
    static public final int B_BUTTON = 2;
    static public final int X_BUTTON = 3;
    static public final int Y_BUTTON = 4;
    static public final int LEFT_BUMPER = 5;
    static public final int RIGHT_BUMPER = 6;
    static public final int BACK_BUTTON = 7;
    static public final int START_BUTTON = 8;
    static public final int LEFT_CLICK = 9;
    static public final int RIGHT_CLICK = 10;
    //
    // Digital Input/Outputs
    //
    static public final int ARM_LIMIT_BACK_ID = 2;
    static public final int ARM_LIMIT_FORWARD_ID = 3;
    static public final int AUTONOMOUS_SWITCH_A = 4;
    static public final int AUTONOMOUS_SWITCH_B = 5;
    //
    // Joystick
    //
    static public final int PilotJoystick = 1;
    static public final int CoPilotJoystick = 2;
    //
    //Other Constants
    //
    static public final int TWELVE_FEET_AWAY = 12;
    static public final double ELEVEN_AND_A_HALF_FEET_AWAY = 11.5;
    static public final int TEN_FEET_AWAY = 10;
    static public final int NINE_FEET_AWAY = 9;

}
