package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Solenoid;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private static Claw instance = null;

    private final DoubleSolenoid solenoid;
    private boolean state; // push is true, and pull is false

    public Claw() { 
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,0, 1);//0 and 1
        this.push();
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
        solenoid.set(DoubleSolenoid.Value.kReverse);
        state = true;
    }

    public void pull() {
        solenoid.set(DoubleSolenoid.Value.kForward);
        state = false;
    }

    public void toggle() {
        System.out.println("ITS ON: " + state);
        if (state) {
            pull();
        } else {
            push();
        }
    }
}
