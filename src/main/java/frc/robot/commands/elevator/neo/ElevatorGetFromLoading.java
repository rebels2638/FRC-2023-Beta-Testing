package frc.robot.commands.elevator.neo;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.NeoElevatorPIDNonProfiled;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Wrist;

import frc.robot.commands.elevator.neo.ElevatorCancel;

public class ElevatorGetFromLoading extends SequentialCommandGroup {
        public ElevatorGetFromLoading() {
                addCommands(
                                new ParallelCommandGroup(
                                                new ParallelRaceGroup(new ElevatorUp(NeoElevatorPIDNonProfiled.getInstance()), new TimerCommand(2.5)),
                                                new InstantCommand(() -> Claw.getInstance().push())),
                                new WristLoading(Wrist.getInstance())
                                , new ElevatorCancel(NeoElevatorPIDNonProfiled.getInstance()));
        }
}