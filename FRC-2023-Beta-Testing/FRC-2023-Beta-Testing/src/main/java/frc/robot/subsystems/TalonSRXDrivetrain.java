package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;
import frc.robot.utils.RobotMap;
import frc.robot.utils.ConstantsSRXDriveTrain.DriveConstants;
import frc.robot.utils.ConstantsSRXDriveTrain.DriveConstants.DriveTypes;

public class TalonSRXDrivetrain extends SubsystemBase {
    private static TalonSRXDrivetrain instance = null;

    private DriveTypes currentDriveType;
    private boolean quickTurn;

    // Left and Right Gearboxes
    private final TalonSRXGearbox gearboxLeft, gearboxRight;

    // Left and Right MotorControllerGroup
    private final MotorControllerGroup leftMotorControllerGroup, rightMotorsControllerGroup;

    // DifferentialDrive
    public static DifferentialDrive m_drive;

    // Skew Rate Limiter - primitive motion profile
    private SlewRateLimiter tankLeftLimiter, tankRightLimiter;

    // -------------------- Start of Shuffleboard things --------------------

    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Debug");
    // -------------------- End of Shuffleboard things --------------------

    public TalonSRXDrivetrain() {
        // gearboxes
        this.gearboxLeft = new TalonSRXGearbox(RobotMap.DRIVE_LEFT_FRONT, RobotMap.DRIVE_LEFT_BACK);
        this.gearboxRight = new TalonSRXGearbox(RobotMap.DRIVE_RIGHT_FRONT, RobotMap.DRIVE_RIGHT_BACK);

        // inversions
        this.gearboxLeft.getFrontTalon().setInverted(DriveConstants.FALCON_LEFT_FRONT_INVERTED);
        this.gearboxLeft.getBackTalon().setInverted(DriveConstants.FALCON_LEFT_BACK_INVERTED);
        this.gearboxRight.getFrontTalon().setInverted(DriveConstants.FALCON_RIGHT_FRONT_INVERTED);
        this.gearboxRight.getBackTalon().setInverted(DriveConstants.FALCON_RIGHT_BACK_INVERTED);

        this.tankLeftLimiter = new SlewRateLimiter(DriveConstants.SLEW_RATE_LIMIT);
        this.tankRightLimiter = new SlewRateLimiter(DriveConstants.SLEW_RATE_LIMIT);

        this.leftMotorControllerGroup = new MotorControllerGroup(this.gearboxLeft.getFrontTalon(),
                this.gearboxLeft.getBackTalon());

        this.rightMotorsControllerGroup = new MotorControllerGroup(this.gearboxRight.getFrontTalon(),
                this.gearboxRight.getBackTalon());

        // -------------------- Testing with Speed Controllers --------------------

        m_drive = new DifferentialDrive(this.leftMotorControllerGroup, this.rightMotorsControllerGroup);

        this.currentDriveType = DriveConstants.DRIVE_TYPE_DEFAULT;
    }

    // Singleton class, call getInstance instead of the constructor (which is
    // private)
    public static TalonSRXDrivetrain getInstance() {
        if (instance == null) {
            instance = new TalonSRXDrivetrain();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // -------------------- Shuffleboard Updates --------------------

    }

    public void defaultDrive(double leftY, double rightY, double rightX) {


        
        leftY = RebelUtil.linearDeadband(leftY, DriveConstants.DEADBAND);
        rightY = RebelUtil.linearDeadband(rightY, DriveConstants.DEADBAND);
        rightX = RebelUtil.linearDeadband(rightX, DriveConstants.DEADBAND);
        
        // * Driving

        // Quick Turn enabled when between threshold
        this.quickTurn = Math.abs(leftY) < DriveConstants.QUICK_TURN_THRESHOLD;

        if (currentDriveType == DriveTypes.CURVATURE) {
            leftY = tankLeftLimiter.calculate(leftY);

            System.out.println(rightX + " " + quickTurn);
            // TODO: Maybe there is a better library for this quadriatic tuning
            rightX *= (rightX < 0 ? 1 : -1) * rightX;
            driveCurvature(leftY, rightX, this.quickTurn);
        } else if (currentDriveType == DriveTypes.TANK) {
            leftY = tankLeftLimiter.calculate(leftY);
            rightY = tankRightLimiter.calculate(rightY);
            driveTank(leftY, rightY);
        } else {
            // TODO: Is it possible to set errors or warnings? If so make the following a warning
            System.out.println("Drive type not set, thus going with curvatureDrive");
            driveCurvature(leftY, rightY, this.quickTurn);
        }
    }

    // -------------- Teleop Driving -----------------------
    public void driveCurvature(double speed, double rotation, boolean isQuick) {
        // TODO: Why is this necessary?
        rotation *= speed < -0.05 ? -1 : 1;
        m_drive.curvatureDrive(speed, rotation, isQuick);
    }

    // ---------- Previous Auto Driving & Tank Drive -------
    public void driveTank(double leftPercent, double rightPercent) {
        m_drive.tankDrive(leftPercent,rightPercent);
    }

    // -------------------- Driving Utility -----------------------
    public void setCurrentDriveType(DriveTypes type) {
        this.currentDriveType = type;
    }
}
