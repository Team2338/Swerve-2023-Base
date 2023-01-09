package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.drivers.Pigeon;
import frc.robot.subsystems.drivers.SwerveModule;

public class Swervetrain extends SubsystemBase {
    public static SwerveModule fL;
    public static SwerveModule fR;
    public static SwerveModule rR;
    public static SwerveModule rL;

    private static TalonSRX pigMotor;
    private static Pigeon pig;

    private static SwerveDriveOdometry odometry;

    /**
     * Constructor for swerve drivetrain using 4 swerve modules using NEOs to drive and TalonSRX to control turning
     */
    public Swervetrain() {
        super();

        fL = new SwerveModule(
                RobotMap.kFrontLeftDriveMotorPort,
                RobotMap.kFrontLeftTurningMotorPort,
                true,
                true,
                false,
                Constants.Drivetrain.kFrontLeftOffset
        );

        fR = new SwerveModule(
                RobotMap.kFrontRightDriveMotorPort,
                RobotMap.kFrontRightTurningMotorPort,
                true,
                true,
                false,
                Constants.Drivetrain.kFrontRightOffset
        );

        rR = new SwerveModule(
                RobotMap.kRearRightDriveMotorPort,
                RobotMap.kRearRightTurningMotorPort,
                true,
                true,
                true,
                Constants.Drivetrain.kRearRightOffset
        );

        rL = new SwerveModule(
                RobotMap.kRearLeftDriveMotorPort,
                RobotMap.kRearLeftTurningMotorPort,
                true,
                true,
                false,
                Constants.Drivetrain.kRearLeftOffset
        );

        //        resetEncoders();
        pigMotor = new TalonSRX(RobotMap.DUMMY);
        pig = new Pigeon(pigMotor);
        odometry = new SwerveDriveOdometry(Constants.Drivetrain.kDriveKinematics, pig.getRotation2d(),
                new SwerveModulePosition[] {
                        fL.getPosition(),
                        rL.getPosition(),
                        fR.getPosition(),
                        rR.getPosition()
                });

        resetHeading();
        resetEncoders();
    }

    @Override
    public void periodic() {
        odometry.update(
                Rotation2d.fromDegrees(-pig.getHeading()),
                new SwerveModulePosition[] {
                        fL.getPosition(),
                        rL.getPosition(),
                        fR.getPosition(),
                        rR.getPosition()
                }

        );
    }


    public void drive(double x, double y, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(
                        fieldRelative ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, pig.getRotation2d()) :
                                new ChassisSpeeds(x, y, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond
        );

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);
    }


    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Field Relative ChassisSpeeds to apply to wheel speeds
     */
    public void setModuleStatesChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond
        );

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Reset the position of each of the wheels so that they all are pointing straight forward
     */
    public void resetEncoders() {
        fL.resetEncoders();
        fR.resetEncoders();
        rL.resetEncoders();
        rR.resetEncoders();
    }

    /**
     * Reset the pigeon heading
     */
    public void resetHeading() {
        pig.resetPigeonPosition();
    }


    /**
     * Get the pigeon heading
     * @return The pigeon heading in degrees
     */
    public double getHeading() {
        return pig.getRotation2d().getDegrees();
    }

    public SwerveModuleState getRRState() {
        return rR.getState();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pig.getRotation2d(),
                new SwerveModulePosition[] {
                        fL.getPosition(),
                        rL.getPosition(),
                        fR.getPosition(),
                        rR.getPosition()
                },
                pose
        );
    }

    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }
}