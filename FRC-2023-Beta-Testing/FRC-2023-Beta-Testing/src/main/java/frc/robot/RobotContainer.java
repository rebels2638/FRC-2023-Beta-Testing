// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.input.XboxController;

import frc.robot.subsystems.TalonSRXDrivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightNT;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.HopperConstants;
import frc.robot.utils.Constants.DriveConstants.DriveTypes;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // ---------- Robot Subsystems ---------- \\
  private final TalonSRXDrivetrain drive = TalonSRXDrivetrain.getInstance();

  // ---------- Robot Subsystems ---------- \\

  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  // Create a Sendable Chooser, which allows us to select between Commands (in
  // this case, auto commands)
  private final SendableChooser<Command> chooser = new SendableChooser<Command>();

//   private final Command lowOneBallAuto = new lowOneBallAuto(drive, shooter, hopper, intake, hood);
//   private final Command highRobustTwoBallAuto = new highRobustTwoBallAuto(drive, shooter, hopper, intake, hood);
//   private final Command highRobustThreeBallAuto = new highRobustThreeBallAuto(drive, shooter, hopper, intake, hood);
//   private final Command lowRobustTwoBallAuto = new lowRobustTwoBallAuto(drive, shooter, hopper, intake, hood);

//   private final Command highOneBallAuto = new highOneBallAuto(drive, shooter, hopper, intake);
//   private final Command highRobustTwoBallAuto = new highRobustTwoBallAuto(drive, shooter, hopper, intake);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxDriver = new XboxController(Constants.XBOX_DRIVER_PORT);
    this.xboxOperator = new XboxController(Constants.XBOX_OPERATOR_PORT);

    // Controler Throttle Mappings
    this.drive.setDefaultCommand(
        new RunCommand(() -> this.drive.defaultDrive(
            this.xboxDriver.getLeftY(), this.xboxDriver.getRightY(),
            this.xboxDriver.getRightX() * DriveConstants.TURN_SENSITIVITY), this.drive));

    // this.shooter.setDefaultCommand(
    //     new RunCommand(() -> this.shooter.setShooterPercentage(
    //         this.xboxOperator.getRightY()), this.shooter));

    // Configure the button bindings
    // configureButtonBindings();

//     // Additional Configurations
//     // drive.resetOdometry(new Pose2d());

//     // chooser.setDefaultOption("HIGH TWO", highRobustTwoBallAuto);
//     // chooser.addOption("HIGH THREE", highRobustThreeBallAuto);
//     // chooser.addOption("LOW TWO", lowRobustTwoBallAuto);
//     // chooser.addOption("ONE BALL", lowOneBallAuto);
// //     chooser.addOption("highOneBallAuto", highOneBallAuto);
// //     chooser.addOption("highRobustTwoBallAuto", highRobustTwoBallAuto);

//     final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Debug");
//     autoTab.add("Auto Choice", chooser);

    
//   }

//   /**
//    * Use this method to define your button->command mappings. Buttons can be
//    * created by
//    * instantiating a {@link GenericHID} or one of its subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
//    * it to a {@link
//    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
//    */
//   private void configureButtonBindings() {
//     // ---------- DRIVER Bindings ---------- \\

//     /**
//      * Left Trigger - Low Goal
//      */
//     this.xboxDriver.getLeftTriggerButton()
//         .whileHeld(new LowGoalShot(shooter, hopper, hood, intake, 999999));

//     this.xboxDriver.getDownDpad()
//         .whenHeld(new RunShooterWheel(shooter));

//     /**
//      * Right Trigger - High Goal with Limelight Tracking
//     //  */
//     // this.xboxDriver.getRightTriggerButton()
//     //     .whileHeld(new HighGoalShot(drive, shooter, hopper, hood, intake));

//     /**
//      * Left Bumper - Switch drivetrain to Low Gear
//      */
//     this.xboxDriver.getLeftBumper()
//         .whenPressed(new InstantCommand(() -> this.drive.shiftToLowGear()));

//     /**
//      * Left Bumper - Switch drivetrain to High Gear
//      */
//     this.xboxDriver.getRightBumper()
//         .whenPressed(new InstantCommand(() -> this.drive.shiftToHighGear()));

    /**
     * Directional Pad
     * Up -
     * Down -
     * Left -
     * Right -
     */
    // this.xboxDriver.getDownDpad().whenPressed(new PerciseTurning(drive, 180,
    // 0.35));
    // this.xboxDriver.getLeftDpad().whenPressed(new PerciseTurning(drive, -90,
    // 0.35));
    // this.xboxDriver.getRightDpad().whenPressed(new PerciseTurning(drive, 90,
    // 0.35));

//     /**
//      * X_Button - Auto Aim Auto Shoot ALL CONDITIONS
//      */
//     this.xboxDriver.getXButton()
//         .whileHeld(new LimelightAimingAuto(drive, shooter, hopper, hood, intake))
//         .whenReleased(new InstantCommand(() -> this.drive.shiftToLowGear()));

// //     /**
// //      * Y_Button - Far high shot
// //      */
//     this.xboxDriver.getYButton()
//         .whileHeld(new LimelightAimingHigh(drive));

//     this.xboxDriver.getBButton()
//         .whileHeld(new LimelightAimingLow(drive));

//     this.xboxDriver.getBButton()
//         .whenReleased(new InstantCommand(() -> this.drive.shiftToLowGear()));

//     // ---------- OPERATOR Bindings ---------- \\
//     /**
//      * Left Trigger -
//      */
    
//     this.xboxOperator.getLeftTriggerButton()
//     .whileHeld(new AutoDown(climber));

//     /**
//      * Right Trigger
//      */

//      this.xboxOperator.getRightTriggerButton()
//      .whileHeld(new AutoUp(climber));

//     /**
//      * A_Button - Automated Hopper Intake Process using beam break sensors
//      */
//     this.xboxDriver.getAButton()
//     .whenHeld(new AutomatedHopperIntakeProcess(intake, hopper));

//      this.xboxOperator.getAButton()
//     .whenHeld(new AutomatedHopperIntakeProcess(intake, hopper));

//     /**
//      * X_Button - Intake pnumatics override
//      */
//     this.xboxOperator.getXButton()
//         .whenPressed(new InstantCommand(() -> this.intake.toggleIntake()));

//     /**
//      * Y_Button - Climb Pistons
//      */
//     this.xboxOperator.getYButton()
//         .whenPressed(new InstantCommand(() -> this.pnumaticClimber.push()))
//         .whenReleased(new InstantCommand(() -> this.pnumaticClimber.pull()));

    
//      /**
//      * Directional Pad
//      * Up - Move Intake and Hopper to move balls up the hopper.
//      * Down - Move Intake and Hopper to move balls down the hopper.
//      * Left -
//      * Right -
//      */
//     this.xboxOperator.getUpDpad()
//         .whenHeld(new InstantCommand(() -> this.intake.setPercentOutput(0.75)))
//         .whenHeld(new InstantCommand(() -> this.hopper.setPercentOutput(0.35)))
//         .whenReleased(new InstantCommand(() -> this.intake.setPercentOutput(0)))
//         .whenReleased(new InstantCommand(() -> this.hopper.setPercentOutput(0)));

//     this.xboxOperator.getDownDpad()
//         .whenHeld(new InstantCommand(() -> this.intake.setPercentOutput(-0.75)))
//         .whenHeld(new InstantCommand(() -> this.hopper.setPercentOutput(-0.35)))
//         .whenReleased(new InstantCommand(() -> this.intake.setPercentOutput(0)))
//         .whenReleased(new InstantCommand(() -> this.hopper.setPercentOutput(0)));
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     this.drive.shiftToLowGear();
//     this.drive.configureForTeleop();

//     Command chosen = chooser.getSelected();
//     this.drive.resetOdometry(new Pose2d());
//     return chosen;
//   }

//   public void switchToLow() {
//       this.drive.shiftToLowGear();
//   }
  }
}
