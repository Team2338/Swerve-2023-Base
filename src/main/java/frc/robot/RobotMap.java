package frc.robot;

public abstract class RobotMap {
    public static final int kFrontLeftDriveMotorPort = 23;
    public static final int kRearLeftDriveMotorPort = 20;
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kRearRightDriveMotorPort = 16;

    public static final int kFrontLeftTurningMotorPort = 5; //not 12
    public static final int kRearLeftTurningMotorPort = 7; //not 8
    public static final int kFrontRightTurningMotorPort = 8;
    public static final int kRearRightTurningMotorPort = 2;

    public static final int DUMMY = 24;


    public static final int PIGEON = 4;

    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
}
