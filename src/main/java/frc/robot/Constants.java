package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static final class Drivetrain {
    //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

    public static final boolean kFrontLeftTurningEncoderReversed = false; //false
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kRearRightDriveMotorReversed = false;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kRearLeftTurningMotorReversed = false;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = false;

    public static final double kFrontLeftOffset = -0.0; // TODO: Calculate
    public static final double kRearLeftOffset = 0.0;
    public static final double kFrontRightOffset = 0.0;
    public static final double kRearRightOffset = 0.0;

    public static final double kTrackWidth = 0.4699;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.4699;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // x was +, y was +
                    new Translation2d(kWheelBase / 2, kTrackWidth / 2), // x was +, y was -
                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // x was -, y was +
                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // x was -, y was -

    public static final boolean kGyroReversed = false;

    public static final double kMaxDriveRPM = 4800;

    public static final double kMaxSpeedMetersPerSecond = kMaxDriveRPM *
            (Math.PI * Constants.ModuleConstants.kWheelDiameterMeters) /
            (60.0 * Constants.ModuleConstants.kGearRatio);

    public static final double kMaxSpeedTurning = Math.PI;
    public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 6 * (2 * Math.PI); //6
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 6 * (2 * Math.PI); //7

    public static final double kDriveMotorGearRatio = 12.8; // TODO: Need to ask Aaron
    public static final double kWheelDiameterMeters = 0.10338;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

    public static final double kEncoderCPR = 4096.0; //1024
    public static final double kFalconEncoderCPR = 2048;
    public static final double kDriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.45; // 1

    public static final double kPModuleDriveController = 0.008; // 1

    public static final double kGearRatio = 46080.0 / 6720.0; // need to ask aaron
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2.2;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class DriveConstants {
    public static final double deadband = 0.05;
  }
}
