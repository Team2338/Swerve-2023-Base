package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drive extends CommandBase {
    public Drive() {
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

        //Forward speed, Sideways speed, Rotation Speed
        ChassisSpeeds xchassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                y * Constants.Drivetrain.kMaxSpeedMetersPerSecond,
                -x * Constants.Drivetrain.kMaxSpeedMetersPerSecond,
                rot * Constants.Drivetrain.kMaxSpeedTurning,
                Rotation2d.fromDegrees(Robot.swervetrain.getHeading())
        );

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(y, -x, rot);

        Robot.swervetrain.drive(
            6.0 * (y * Math.abs(y)),
            6.0 * (x * Math.abs(x)),
            4.0 * rot,
            false
        );
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
