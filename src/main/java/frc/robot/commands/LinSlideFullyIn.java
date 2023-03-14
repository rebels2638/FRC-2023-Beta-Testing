package frc.robot.commands;

import frc.lib.input.XboxController;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LinSlideFullyIn extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LinearSlide m_linslide;
    private final LinSlidePiston m_piston;
    public boolean finished = false;
    public int minEncoderVelo = 3000;

    public LinSlideFullyIn(LinearSlide linslide, LinSlidePiston piston) {
        m_linslide = linslide;
        m_piston = piston;
        finished = false;

        addRequirements(linslide, piston);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // follow position control to goal state
        finished = false;
        m_piston.pull();
        m_linslide.setPercentOutput(-0.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Math.abs(m_linslide.getCurrentEncoderRate()) < minEncoderVelo && m_linslide.getCurrentEncoderPosition() < 10000) {
            m_piston.push();
            m_linslide.setPercentOutput(0.0);
            finished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_linslide.setPercentOutput(0.0);
        m_linslide.zeroEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
