package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.Constants.GearboxConstants;

/**
 * Gear box controller. The solenoid shifts gears by inserting or removing a
 * gear.
 */

public class TalonSRXGearbox {
    private final List<WPI_TalonSRX> talons;

    public TalonSRXGearbox(WPI_TalonSRX frontTalon, WPI_TalonSRX backTalon) {
        this.talons = List.of(frontTalon, backTalon);

        // configureTalons();
    }

    public TalonSRXGearbox(int frontId, int backId) {
        this(new WPI_TalonSRX(frontId), new WPI_TalonSRX(backId));
    }

    public void configureTalons() {
        TalonSRXConfiguration falconConfig = new TalonSRXConfiguration();

        falconConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        falconConfig.slot0.kP = GearboxConstants.MOTION_P;
        falconConfig.slot0.kI = GearboxConstants.PID_I;
        falconConfig.slot0.kD = GearboxConstants.PID_D;
        falconConfig.slot0.kF = GearboxConstants.PID_F;
        falconConfig.voltageCompSaturation = GearboxConstants.VOLTAGE_COMP_SATURATION;

        this.talons.forEach(talon -> {
            talon.configAllSettings(falconConfig);
            talon.setNeutralMode(GearboxConstants.DEFAULT_NEUTRAL_MODE);
            talon.enableVoltageCompensation(GearboxConstants.VOLTAGE_COMP_ENABLED);
        });
    }

    public void setFollowers() {
        this.getBackTalon().follow(this.getFrontTalon());
    }

    public WPI_TalonSRX getFrontTalon() {
        return talons.get(0);
    }

    public WPI_TalonSRX getBackTalon() {
        return talons.get(1);
    }
}