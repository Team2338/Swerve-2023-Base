package frc.robot.subsystems.drivers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.List;

public abstract class SwerveTraj implements Command {

    public Command tComm;

    /**
     * Make a trajectory
     * @param waypoints Each of the waypoints (x, y) as Pose2D List
     * @param fPoint Final coordinate to route to enroute
     * @param fRot Final rotation to rotate to enroute
     */
    public SwerveTraj(List <Translation2d> waypoints, Translation2d fPoint, Rotation2d fRot) {
        super();

        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Drivetrain.kDriveKinematics);
        //sets new config object, holds trajectory information for reference

        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                waypoints,
                new Pose2d(fPoint, fRot),
                config
                //generates a trajectory for SwerveTraj class
        );

        //sda
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory1,
                Robot.swervetrain::getPose,
                Constants.Drivetrain.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                Robot.swervetrain::setModuleStates,
                Robot.swervetrain
        );

        Robot.swervetrain.resetOdometry(trajectory1.getInitialPose());

        tComm = swerveControllerCommand.andThen(() -> Robot.swervetrain.drive(0, 0, 0, false));
    }
}