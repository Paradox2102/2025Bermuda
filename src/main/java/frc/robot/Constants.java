// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double k_robotMass = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter k_chassis    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), k_robotMass);
  public static final double k_loopTime  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double k_maxSpeed  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double k_wheelLockTime = 10; // seconds
  }

  public static final class IntakePivotConstants {
    public static final double k_resetPosition = 0;
    public static final double k_deadzone = 0;

    public static final double k_gearRatio = 11.67;
    public static final double k_momentOfInertia = 0.485;
    public static final double k_armLengthMeters = 0.5;

    public static final double k_f = 0;
    public static final double k_p = 0;
    public static final double k_i = 0;
    public static final double k_d = 0;

    public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
    static {
        pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        pivotConfig.inverted(false);
        pivotConfig.absoluteEncoder
              .positionConversionFactor(360)
              .zeroOffset(k_resetPosition);
    }
  }

  public static class ElevatorConstants {
    public static final double k_deadzone = 0;
    public static final double k_posConversionFactor = 0;

    public static final double k_p = 0;
    public static final double k_i = 0;
    public static final double k_d = 0;
    //found by reca.lc linear mechanism model
    public static final double k_g = 0.38;
    public static final double k_v = 4.69;
    public static final double k_a = 0.06;
    public static final double k_maxAccel = 42.15;
    public static final double k_maxVel = 2.45;

    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig followConfig = new SparkFlexConfig();
    static {
      elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
      elevatorConfig.inverted(false);
      elevatorConfig.encoder.positionConversionFactor(k_posConversionFactor).velocityConversionFactor(k_posConversionFactor);
      followConfig.apply(elevatorConfig).inverted(true);
      followConfig.follow(CANIDConstants.elev_leader, true);
    }
  }

  public static class ArmConstants {
    public static final double k_deadzone = 0;
    public static final double k_resetPosition = 0;

    public static final double k_gearRatio = 27;
    public static final double k_momentOfInertia = 0.308;
    public static final double k_armLengthMeters = 0.543;

    public static final double k_p = 0;
    public static final double k_i = 0;
    public static final double k_d = 0;
    public static final double k_f = 0;

    public static final SparkFlexConfig armConfig = new SparkFlexConfig();
    static {
        armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        armConfig.inverted(false);
        armConfig.absoluteEncoder
              .positionConversionFactor(360)
              .zeroOffset(k_resetPosition);
    }
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double k_deadBand        = 0.1;
    public static final double k_leftYDeadBand = 0.1;
    public static final double k_rightXDeadBand = 0.1;
    public static final double k_turnConstant    = 6;
  }

  public static class CANIDConstants {
    public static final int gyro = 0;
    public static final int fl_drive = 1;
    public static final int fl_turn = 2;
    public static final int fr_drive = 3;
    public static final int fr_turn = 4;
    public static final int bl_drive = 5;
    public static final int bl_turn = 6;
    public static final int br_drive = 7;
    public static final int br_turn = 8;
    public static final int elev_leader = 10;
    public static final int elev_follower = 11;
    public static final int arm = 12;
    public static final int claw = 13;
    public static final int intake_pivot = 20;
    public static final int intake_roller = 21;
    public static final int climber = 30;
    public static final int climber_follow = 31;
    public static final int cage_grabber = 32;
  }
}