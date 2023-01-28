package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmPIDandFeedForward extends SubsystemBase {
    private final WPI_TalonFX talon;
    private static Arm instance = null;
    private static double lastPercentSpeed; 
  
  
    private static final double kS = 0.0;
    private static final double kG = 0.0;
    private static final double kV = 0.0;
    private static final double kA = 0.0;
    
    
    private static ArmFeedforward m_feedforward;

    private static final double kStartAngle = -23.0;

    private static final double kEncoderResolution = 4096;

    private static final double kVelocityMultiplier = 1.0;

    private static double kAngle = kStartAngle;
    private static  double kAngleIncrement;
    private static double kPreviosPosition;

    public ArmPIDandFeedForward() {
        this.talon = new WPI_TalonFX(5); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        lastPercentSpeed = 0;
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();

        falconConfig.slot0.kP = 0;
        falconConfig.slot0.kI = 0;
        falconConfig.slot0.kD = 0;

        falconConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        falconConfig.voltageCompSaturation = 12;

        falconConfig.nominalOutputForward = 0;
        falconConfig.nominalOutputReverse = 0;
        falconConfig.peakOutputForward = 1;
        falconConfig.peakOutputReverse = -1;

        talon.configAllSettings(falconConfig);
        talon.setNeutralMode(NeutralMode.Coast);
        
        // kS and kG should have units of volts, kV should have units of volts * seconds / radians, and kA should have units of volts * seconds^2 / radians.
        // Units must be consistent! TODO: get feedfowrward gains tuned
      
        m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
        
        kAngleIncrement = 360 / kEncoderResolution;
        
        kPreviosPosition = talon.getSelectedSensorPosition();
        
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void setPercentVoltage(double percentSpeed) {
        
        double distance = talon.getSelectedSensorPosition() - kPreviosPosition;
        kAngle += distance * kAngleIncrement;
        kPreviosPosition = talon.getSelectedSensorPosition();
        
        // second arg is in rad/sec
        double feedOut = m_feedforward.calculate(kAngle * (Math.PI / 180), percentSpeed * kVelocityMultiplier);

        talon.setVoltage(feedOut);
    }
    
}
