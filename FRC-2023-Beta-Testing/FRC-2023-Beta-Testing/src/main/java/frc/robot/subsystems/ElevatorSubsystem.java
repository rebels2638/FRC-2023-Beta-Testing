package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;

public class Elevator extends SubsystemBase {

    private final WPI_TalonSRX talon;
    private static Elevator instance = null; 

    private Elevator() {
        this.talon = new WPI_TalonSRX(0); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void setPercentOutput(double speed) {
      
        talon.set(speed); // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed???? where speed is in range -1,1 and the xbox controller left joy stick is also -1,1???
      
    }
}
