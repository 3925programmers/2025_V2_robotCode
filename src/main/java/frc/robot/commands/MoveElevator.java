package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command{
    private final Elevator m_elevator;
    private final double m_setpoint;

    public MoveElevator(Elevator elevator, double setpoint) {
        m_elevator = elevator;
        m_setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.Travel(m_setpoint); // Command the elevator to the target position
    }

    @Override
    public boolean isFinished() {
        return m_elevator.getState() == Elevator.ElevatorState.Holding; // Command finishes when elevator is holding
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_elevator.Rest(); // Stop the elevator if the command is interrupted
        }
    }

}