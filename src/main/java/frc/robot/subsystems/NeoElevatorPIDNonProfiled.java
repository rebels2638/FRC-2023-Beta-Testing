package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.ConstantsArmElevator.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

/** Elevator subsystem with feed-forward and PID for position */
public class NeoElevatorPIDNonProfiled extends SubsystemBase {
    private static NeoElevatorPIDNonProfiled instance = null;

    public static final double kMaxSpeed = 1.5; // meters per second
    public static final double kMaxAcceleration = 1.7; // meters per second squared

    private static final double kWheelRadius = 0.018191; // meters
    private static final int kEncoderResolution = 2048; 
    private static final int kGearingRatio = 6;

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static  final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;
    
    private final CANSparkMax m_motor1 = new CANSparkMax(1,MotorType.kBrushless); // TODO: change IDs
    private final CANSparkMax m_motor2 = new CANSparkMax(2,MotorType.kBrushless);

    private final PIDController m_controller = new PIDController(12, 0, 0);
    private final ProfiledPIDController m_profiledController = new ProfiledPIDController(12, 0, 0, new TrapezoidProfile.Constraints(1.57268, 22.1216));
    private final PIDController m_velocityController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    public boolean m_velocityControlEnabled = true;

    private double m_velocitySetpoint = 0;
    private double m_voltageSetpoint = 0;
    private double m_heightAccumulator = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastVelocity = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    private static double kUpperLimit = 0.75;
    private static double kLowerLimit = -0.2;

    private final ShuffleboardTab tab;
    /**
     * Commented out because it kept causing duplicacy issues during runtime 
     * (it's just duplicate entries)
     */
    private  final GenericEntry elevatorEncoderPosition;
    private  final GenericEntry elevatorPosition;
    private  final GenericEntry elevatorVelocity;
    private  final GenericEntry elevatorAcceleration;
    private  final GenericEntry elevatorPositionSetpoint;
    
    private  final GenericEntry elevatorVelocitySetpoint;
    private  final GenericEntry elevatorAccelerationSetpoint;
    private  final GenericEntry voltageSupplied;
    private  final GenericEntry voltageSetpoint;

    public NeoElevatorPIDNonProfiled() {
        m_motor1.setInverted(false); // they changed the motor
        m_motor2.setInverted(false);

        // reset elevator
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //zeroEncoder();
        m_motor1.getEncoder().setVelocityConversionFactor(kMetersPerRotation/60);
        m_motor1.getEncoder().setPositionConversionFactor(kMetersPerRotation/60);


        // m_motor1.set(ControlMode.PercentOutput, 0); Hypothesis: You dont need these :pray:
        // m_motor2.set(ControlMode.PercentOutput, 0); TODO: change the command to instantiate an object of this instead
        setToVelocityControlMode(true);
        setGoal(0);
        setVelocitySetpoint(0);
        resetHeightAccumulator();
        m_controller.setTolerance(0.07, 0.1);
        
        tab = Shuffleboard.getTab("Elevator");
        elevatorEncoderPosition = tab.add("Encoder Position", 0.0).getEntry();
        elevatorPosition = tab.add("Height", 0.0).getEntry();
        elevatorVelocity = tab.add("Velocity", 0.0).getEntry();
        elevatorAcceleration = tab.add("Acceleration", 0.0).getEntry();
        elevatorPositionSetpoint = tab.add("Height Setpoint", 0.0).getEntry();
        elevatorVelocitySetpoint = tab.add("Velocity Setpoint", 0.0).getEntry();
        elevatorAccelerationSetpoint = tab.add("Acceleration Setpoint", 0.0).getEntry();
        voltageSupplied = tab.add("Motor Voltage", 0.0).getEntry();
        voltageSetpoint = tab.add("Voltage Setpoint", 0.0).getEntry();

        // tab.add("Zero Encoder",
        //         new InstantCommand(() -> this.zeroEncoder()));
        // zeroEncoder();
    }

    public static NeoElevatorPIDNonProfiled getInstance() {
        if (instance == null) {
            instance = new NeoElevatorPIDNonProfiled();
        }
        return instance;
    }

    /*
    * Convert from TalonFX elevator position in meters to native units and vice versa
    */
    public double heightToNative(double heightUnits) {
        return heightUnits * kRotationsPerMeter * kNativeUnitsPerRotation;
    }

    public double nativeToHeight(double encoderUnits) {
        
        return encoderUnits * kRotationsPerNativeUnit * kMetersPerRotation;
    }

    public void setGoal(double height) {
        m_controller.setSetpoint(height);
    }

    public boolean atGoal() {
        return m_controller.atSetpoint();
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        m_velocitySetpoint = velocitySetpoint;
    }

    public void setToVelocityControlMode(boolean on) {
        m_velocityControlEnabled = on;
        resetHeightAccumulator();
    }

    public void resetHeightAccumulator() {
        m_heightAccumulator = getCurrentHeight();
    }
    //This one is in RPM(Rotations Per Minute)
    public double getCurrentEncoderRate() {
        return m_motor1.getEncoder().getVelocity(); // motor velocity is in ticks per 100ms
    }

    public double getCurrentHeight() {
        return m_motor1.getEncoder().getPosition();
    }

    public double getCurrentVelocity() {
        return m_motor1.getEncoder().getVelocity();/* conversion factor */
    }

    public double getCurrentAcceleration() {
        return (getCurrentVelocity() - m_lastVelocity) / (Timer.getFPGATimestamp() - m_lastTime);
    }

    // public void zeroEncoder() {
    //     m_motor1.getSensorCollection().setIntegratedSensorPosition(0, 30);
    //     m_motor2.getSensorCollection().setIntegratedSensorPosition(0, 30);
    // }

    public void updateShuffleboard() {
        //elevatorEncoderPosition.setDouble(getCurrentEncoderPosition());
        elevatorPosition.setDouble(getCurrentHeight());
        elevatorVelocity.setDouble(getCurrentVelocity());
        elevatorAcceleration.setDouble(getCurrentAcceleration());
        //voltageSupplied.setDouble(m_motor1.getMotorOutputVoltage());
        voltageSetpoint.setDouble(m_voltageSetpoint);
        elevatorPositionSetpoint.setDouble(m_controller.getSetpoint());
    }
    // public boolean isWithinTreshold(double height, double tolerance){
    //     if(Math.abs(getCurrentHeight() - height) < tolerance){
    //         return true;
    //     }
    //     else{
    //     return false;
    //     }
    // }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        // double m_error = m_controller.getSetpoint() - getCurrentHeight();
        double velocityPID = m_velocitySetpoint * 5;
        double positionPID = m_controller.calculate(getCurrentHeight());
        double pid = m_velocityControlEnabled ? velocityPID : positionPID;
        double feedforward = ElevatorConstants.kG + (pid == 0 ? 0 : pid < 0 ? -1 : 1) * ElevatorConstants.kS;
        double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);
        // System.out.println(m_velocityControlEnabled + " " + voltage);
        //System.out.println("VOLTAGE " + voltage);
        if (getCurrentHeight() >= kUpperLimit && voltage >= ElevatorConstants.kG) {
            voltage = ElevatorConstants.kG;

        } else if (getCurrentHeight() <= kLowerLimit && voltage < 0.0) {
            voltage = 0.0;
        } else if(LinearSlide.getInstance().getCurrentEncoderPosition() > 15000) {
            voltage = ElevatorConstants.kG;}
        // } else if(Math.abs(getCurrentHeight() - m_controller.getSetpoint()) < 0.006) {
            // voltage = ElevatorConstants.kG;
        // }

        // System.out.println("Elevator : " + voltage);
        m_voltageSetpoint = RebelUtil.constrain(voltage,-12, 12);
        m_motor1.setVoltage(m_voltageSetpoint);
        m_motor2.setVoltage(m_voltageSetpoint);

        updateShuffleboard();
        m_lastVelocity = getCurrentVelocity();
        m_lastTime = Timer.getFPGATimestamp();
    }

    public void breakMotor() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }
}
