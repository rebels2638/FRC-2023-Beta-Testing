// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;
// import frc.lib.input.XboxController;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// /** An example command that uses an example subsystem. */
// public class midRodScore extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final Arm m_armSubsystem;
//   private final Elevator m_elevatorSubsystem;
//   private final XboxController e_controller; 
//   private boolean lastToggle;
//   private boolean armReachedPosition;
//   private boolean elevatorReachedPosition;
  
//   private final double armMediumStick = Math.PI; // (arbitrary rn) target angle of arm in radians
//   private final double elevatorMediumStick = 0; // (arbitrary rn) target height of elevator in meters

//   public midRodScore(Arm armSubsystem, Elevator elevatorSubsystem, XboxController controller) {
//     e_controller = controller;
//     m_armSubsystem = armSubsystem;
//     m_elevatorSubsystem = elevatorSubsystem;
//     lastToggle = false;
//     armReachedPosition = true;
//     elevatorReachedPosition = true;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_armSubsystem, m_elevatorSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // reset encoder here!!!!
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // yay variables to make my life harder
//     double armEncoderValue = m_armSubsystem.getEncoderValue();
//     double armEncoderValueInRadians = armEncoderValue * ((2 * Math.PI)/2048); // convert from encoder values to radians
//     double elevatorEncoderValue = m_elevatorSubsystem.getEncoderValue();
//     double elevatorEncoderValueInRadians = elevatorEncoderValue * ((2 * Math.PI)/2048);

//     if (e_controller.getBButton().getAsBoolean()) {

//         m_elevatorSubsystem.setPercentOutput(0); // change value when found
//         m_armSubsystem.setPercentOutput(0); // change value when found
        
//     }
    
//     // middle cone placing rod is A button
//     if (e_controller.getYButton().getAsBoolean() && !lastToggle) {
//       lastToggle = true;
//       armReachedPosition = false;
//       elevatorReachedPosition = false;
//     } 
//     if(!e_controller.getYButton().getAsBoolean()) {
//       lastToggle = false;
//     }
    
//     if (!armReachedPosition)
//     {
//       m_armSubsystem.setPercentOutput(0.5); // random percent speed to set it at
//       if (armEncoderValueInRadians == armMediumStick)
//       {
//         armReachedPosition = true;
//         m_armSubsystem.setPercentOutput(0.0);
//       }
//     }
//     if (!elevatorReachedPosition)
//     {
//       m_elevatorSubsystem.setPercentOutput(0.5); // random percent speed to set it at
//       if (elevatorEncoderValueInRadians == elevatorMediumStick)
//       {
//         elevatorReachedPosition = true;
//         m_elevatorSubsystem.setPercentOutput(0.0);
//       }
//     }
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
 