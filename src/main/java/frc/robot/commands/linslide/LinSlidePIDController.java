// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.linslide;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LinSlidePID;

import frc.lib.RebelUtil;
import frc.lib.input.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class LinSlidePIDController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LinSlidePID m_linSlidePID;
  private final XboxController e_controller; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */       
  public LinSlidePIDController(LinSlidePID linSlidePIDSubsystem, XboxController controller) {
    e_controller = controller;
    m_linSlidePID = linSlidePIDSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_linSlidePID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_linSlidePID.setToVelocityControlMode(true);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredVelo = RebelUtil.linearDeadband(e_controller.getRightX(), 0.05) * LinSlidePID.kMaxSpeed;
    // System.out.println(desiredVelo);
    m_linSlidePID.setVelocitySetpoint(desiredVelo);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
