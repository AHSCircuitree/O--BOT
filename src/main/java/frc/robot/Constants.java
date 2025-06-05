// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class Channels {
    public static final int pivotEncoderChannel = -1;
  }
  public static final class DriveConstants {

    public static final double kMaxSpeedMetersPerSecond = 1.7; // prev: 4.8
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration = 4;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(16.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(16.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 17;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 11;

    public static final int kFrontLeftTurningCanId = 16; //16 - 10
    public static final int kRearLeftTurningCanId = 10; //10 - 12
    public static final int kFrontRightTurningCanId = 14; //14 - 16
    public static final int kRearRightTurningCanId = 12; //12 - 14

    public static final int gyroCanId = 41;
    public static final boolean kGyroReversed = false;
    public static final double kMaxAcceleration = 1.85;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 6380 / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

        public static final double kS = 0.034;
        public static final double kV = 0.3;
        public static final double kA = 0.018;

        public static final double kMaxSpeedMetersPerSecond = 4.8;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.08;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.7; //max 6.0 m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class pivot { 
    public static final int PivotMotorID = 25;
    public static final double pivotP = 0.1;
    public static final double pivotI = 0;
    public static final double pivotD = 0;

    public static final double pivotOffset = 0;
    public static final double pivotThreshold = 6;
    public static final double upperPivotThreshold = 60;
  }

  public static final class MotorConstants {
    public static final double frOffset = 0;
    public static final double flOffset = 0;
    public static final double rrOffset = 0;
    public static final double rlOffset = 0;
  }
}