package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private static Claw instance = null; 
    
    private final DoubleSolenoid solenoid;
    private boolean state; // push is true, and pull is false
    
    //void? doesnt have return statement
    public Elevator() {
        // this.victor = new VictorSPX(0); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        
        this.solenoid = new DoubleSolenoid(IntakeConstants.SOLENOID_MODULE_TYPE, IntakeConstants.SOLENOID_FORWARD,
                IntakeConstants.SOLENOID_REVERSE);
      
        state = true;
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }
  
    public void push() {
        // prevent duplicating run
        if (intakeOut) {
            return;
        }

        solenoid.set(DoubleSolenoid.Value.kReverse);
        intakeOut = true;
    }

    public void pull() {
        // prevent duplicating run
        if (!intakeOut) {
            return;
        }
      
        solenoid.set(DoubleSolenoid.Value.kForward);
        intakeOut = false;
    }
} //you forgot this
