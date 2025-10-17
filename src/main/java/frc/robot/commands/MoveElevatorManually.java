package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Direction;

public class MoveElevatorManually extends Command{
    private final Elevator m_elevator;
    private final Direction m_direction;

    public MoveElevatorManually(Elevator elevator, Direction direction) {
        m_elevator = elevator;
        m_direction = direction;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.MoveManually(m_direction); // Command the elevator in the target direction
    }

    @Override
    public boolean isFinished() {
        return m_elevator.getState() == Elevator.ElevatorState.Holding; // Command finishes when elevator is holding
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.Hold();
        if (interrupted) {
 // Stop the elevator if the command is interrupted
        }
    }

}
