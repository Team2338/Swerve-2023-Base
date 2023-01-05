package frc.robot.subsystems.drivers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    private WPI_TalonSRX turnMotor;
    private CANSparkMax driveMotor;

    private boolean isAbsInverted;
    private double turningOffset;

    private final PIDController drivePID = new
            PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);

    private final ProfiledPIDController turningPID = new
            ProfiledPIDController(Constants.ModuleConstants.kPModuleTurningController, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, Constants.ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared
            ));

    /**
     * Constructor for a TalonSRX, NEO based Swerve Module
     * @param driveMotor NEO motor channel ID
     * @param turnMotor TalonSRX motor channel ID
     * @param isTurningInverted Boolean for if the motor turning the axle is inverted
     * @param isDriveInverted Boolean for if the motor driving the wheel is inverted
     * @param isAbsInverted Boolean for if the absolute encoder checking turn position is inverted
     * @param turningOffset Difference between the absolute encoder and the encoder on the turnMotor
     */
    public SwerveModule(
            int driveMotor,
            int turnMotor,
            boolean isTurningInverted,
            boolean isDriveInverted,
            boolean isAbsInverted,
            double turningOffset
    ) {
        this.driveMotor = new CANSparkMax(driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new WPI_TalonSRX(turnMotor);

        this.driveMotor.restoreFactoryDefaults();
        this.turnMotor.configFactoryDefault();

        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast); //TODO: Need confirmation on mode
        this.turnMotor.setNeutralMode(NeutralMode.Brake);

        this.driveMotor.setInverted(isDriveInverted);
        this.turnMotor.setInverted(isTurningInverted);
        this.isAbsInverted = isAbsInverted;

        this.turningOffset = turningOffset;

        this.driveMotor.getEncoder().setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
        this.driveMotor.getEncoder().setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.turnMotor.configSelectedFeedbackCoefficient(1);

        this.turningOffset = turningOffset;

        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Get the active state of the swerve module
     * @return Returns a SwerveModuleState of the drive velocity and turn velocity
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnVelocity()));
    }

    /**
     * Get the active drive velocity
     * @return Returns the active drive velocity as a double in RPM
     */
    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    /**
     * Get the active turn velocity
     * @return Returns the active turn velocity as a double in EncoderTicks per 100ms
     */
    public double getTurnVelocity() {
        return turnMotor.getSelectedSensorVelocity();
    }

    /**
     * Get the raw sensor heading
     * @return Returns the raw position of the turn motor as a double in EncoderTicks
     */
    public double getRawHeading() {
        return turnMotor.getSelectedSensorPosition();
    }

    /**
     * Get the heading of the swerve module
     * @return Returns the heading of the module in radians as a double
     */
    public double getTurningHeading() {
        double heading = turnMotor.getSelectedSensorPosition() * (isAbsInverted ? -1.0 : 1.0);
        heading *= turnMotor.getInverted() ? -1.0 : 1.0;
        heading *= (2.0 * Math.PI) / Constants.ModuleConstants.kEncoderCPR;
        //        heading -= turningOffset;
        return heading;
    }

    /**
     * Set the desired state of the swerve module
     * @param state The desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState state) {

        // Modules will pick the closest equivalent angle, reversing drive motors if necessary
        var stateOptimized = SwerveModuleState.optimize(state,
                new Rotation2d(getTurningHeading()));

        // Calculate the drive output from the drive PID controller.
        final
        var driveOutput =
                drivePID.calculate(getDriveVelocity(), stateOptimized.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final
        var turnOutput =
                turningPID.calculate(getTurningHeading() + turningOffset, stateOptimized.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
    }

    /**
     * Zeros all the SwerveModule encoders
     */
    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turnMotor.setSelectedSensorPosition(0.0, 0, 0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getEncoder().getPosition() * Constants.ModuleConstants.kDriveEncoderRot2Meter * Math.PI * Constants.ModuleConstants.kWheelDiameterMeters,
                new Rotation2d(getTurningHeading()));
    }
}
