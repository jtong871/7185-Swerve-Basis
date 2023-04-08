// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {                               //configure all gunner button binding IDs here
    public static final int kDriverPort = 0;
    public static final int kGunnerPort = 1;
    public static final int kStart = 10;
    public static final int kBack = 9;
  }

  public static class ModuleConstants {                                 //modify these as needed depending on the swerve modules used
    public static final double kP = 0.05;    //needs to be tweaked
    
    public static final double maxSpeed = Units.feetToMeters(5);
    public static final double maxTurnRate = Units.rotationsToRadians(90);
  }

  public static class ChassisConstants {                                //modify these as needed depending on the chassis design/swerve modules used
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveGearRatio = 1 / 6.67;
    public static final double turnGearRatio = 40 / 71;
    
    public static final double trackWidth = Units.inchesToMeters(26);
    public static final double wheelBase = Units.inchesToMeters(26);

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(                //Specifies the positions of each swerve module for WPI libraries to account for
      new Translation2d(wheelBase / 2, -trackWidth / 2),                                              //Needed for swerve to work properly, varies by chassis dimensions
      new Translation2d(wheelBase / 2, trackWidth / 2),
      new Translation2d(-wheelBase / 2, -trackWidth / 2),
      new Translation2d(-wheelBase / 2, trackWidth / 2));
  }

  public static class DriverConstants {                                                                //configure to driver specifications 
    public static final double deadband = 0.01;
    public static final double maxAccel = 0.5;
    public static final double maxTurnAccel = 0.6;
    public static final int kStart = 10;
    public static final int kBack = 9;
  }
}
