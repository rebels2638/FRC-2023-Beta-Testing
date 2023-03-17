// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinSlidePID;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.GenericEntry;

/** An example command that uses an example subsystem. */
public class ToggleEncoderLimits extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist wrist;
  private final ElevatorPIDNonProfiled elevator;
  private final LinearSlide linSLide;
  private boolean encodersOn = true;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Encoders");
  
  private final GenericEntry encoderStatus;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ToggleEncoderLimits(Wrist wrist, ElevatorPIDNonProfiled elevator, LinearSlide linSLide) {
    this.wrist = wrist;
    this.elevator = elevator;
    this.linSLide = linSLide;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(linSLide);
    addRequirements(elevator);
    addRequirements(wrist);

    encoderStatus = tab.add("Encoders On", true).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderStatus.setBoolean(encodersOn);
    
  }

  public void toggle(){
    if (encodersOn) {
      linSLide.disableEncoderLimits();
      elevator.disableEncoderLimits();
      wrist.disableEncoderLimits();
    }
    else {
      linSLide.enableEncoderLimits();
      elevator.enableEncoderLimits();
      wrist.enableEncoderLimits();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
