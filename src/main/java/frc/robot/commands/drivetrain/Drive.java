package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drive extends CommandBase {
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public Drive() {
        this.xLimiter = new SlewRateLimiter(Constants.ModuleConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.ModuleConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.ModuleConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(Robot.swervetrain);
    }

    @Override
    public void initialize() {
        System.out.println("INITIALIZED DRIVE COMMAND");
    }

    @Override
    public void execute() {
        double x = Robot.oi.driver.getLeftX();
        x = (Math.abs(x) > Constants.DriveConstants.deadband) ? x : 0;
        double y = Robot.oi.driver.getLeftY();
        y = (Math.abs(y) > Constants.DriveConstants.deadband) ? y : 0;
        double rot = Robot.oi.driver.getRightX();
        rot = (Math.abs(rot) > Constants.DriveConstants.deadband) ? rot : 0;


       x = xLimiter.calculate(x) * Constants.ModuleConstants.kTeleDriveMaxSpeedMetersPerSecond;
       y = yLimiter.calculate(y) * Constants.ModuleConstants.kTeleDriveMaxSpeedMetersPerSecond;
       rot = turningLimiter.calculate(rot) * Constants.ModuleConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x, y, rot);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swervetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
