package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    SparkMax left_master;
    SparkMax right_follower;

    private double setpoint = 0;
    private double holdPosition = 0;
    private double manualModifier = 0;
    
    ElevatorFeedforward feedforward = new ElevatorFeedforward(
        Constants.ElevatorConstants.Control.kS, Constants.ElevatorConstants.Control.kG, Constants.ElevatorConstants.Control.kV); //holdPosition);
    ProfiledPIDController controller = new ProfiledPIDController(
        Constants.ElevatorConstants.Control.kP, Constants.ElevatorConstants.Control.kI, Constants.ElevatorConstants.Control.kD, 
        new Constraints(Constants.ElevatorConstants.Control.CRUISE_VELOCITY, Constants.ElevatorConstants.Control.ACCELERATION));

    private double output = 0;

    private ElevatorState state;

    public Elevator() {
        left_master = new SparkMax(Constants.ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    
        right_follower = new SparkMax(Constants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(Constants.ElevatorConstants.IDLE_MODE);
            config.inverted(Constants.ElevatorConstants.INVERTED);
            config.smartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT);
            // config.closedLoop.pid(Constants.Elevator.MMConfig.kP, Constants.Elevator.MMConfig.kI, Constants.Elevator.MMConfig.kD);
            // config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
            // config.closedLoop.maxMotion.maxAcceleration(Constants.Elevator.MMConfig.ACCELERATION);
            // config.closedLoop.maxMotion.maxVelocity(Constants.Elevator.MMConfig.CRUISE_VELOCITY);
            config.encoder.velocityConversionFactor(Constants.ElevatorConstants.VEL_CONVERSION);
            config.encoder.positionConversionFactor(Constants.ElevatorConstants.POS_CONVERSION);
            
            left_master.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followConfig = new SparkMaxConfig();
            followConfig.idleMode(Constants.ElevatorConstants.IDLE_MODE);
            followConfig.inverted(Constants.ElevatorConstants.INVERTED);
            followConfig.smartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT);  
            followConfig.follow(left_master);      
        right_follower.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Rest();
    }

    public ElevatorState getState() {
        return state;
    }

    public void Travel(double setpoint) {
        this.setpoint = setpoint;
        if (!isMovingToBounds()) { //Check if the setpoint is out of bounds
            this.setpoint = (setpoint < Constants.ElevatorConstants.LOWER_LIMIT) ? Constants.ElevatorConstants.LOWER_LIMIT : Constants.ElevatorConstants.UPPER_LIMIT;
        }
        state = ElevatorState.Traveling;
    }

    public void Hold() {
        holdPosition = setpoint;
        state = ElevatorState.Holding;
    }

    public void VoltageTesting(double volts) {
        left_master.setVoltage(volts);
        state = ElevatorState.Manual;
    }
    
    public void Rest() {
        state = ElevatorState.Resting;
        left_master.set(0);
    }

    public void MoveManually(Direction direction) {
        switch(direction) {
            case Up:
                manualModifier = 1;
                break;
            case Down:
                manualModifier = -1;
                break;
        }
        state = ElevatorState.Manual;
    }

    @Override
    public void periodic() {
        double desiredVelocity = controller.getSetpoint().velocity;
        double feedforward_output = feedforward.calculate(desiredVelocity);

        if (state != ElevatorState.Manual && Math.abs(left_master.getEncoder().getPosition() - setpoint) < Constants.ElevatorConstants.POSITION_TOLERANCE) { 
            Hold();
            if(setpoint == 0) {
                Rest();
            }
        }

        switch (state) {
            case Resting:
                left_master.set(0);
                break;
            case Traveling:
                output = controller.calculate(left_master.getEncoder().getPosition(), setpoint);
                left_master.set(output+feedforward_output);
                break;
            case Holding:
                output = controller.calculate(left_master.getEncoder().getPosition(), holdPosition);
                left_master.set(output+feedforward_output);
                break;
            case Manual:
                left_master.set(manualModifier * 0.3 + feedforward_output);
                break;
            default:
                break;
        }
        SmartDashboard.putString("Current Elevator State", state.name());
        SmartDashboard.putNumber("Elevator Position", left_master.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Motor Output", left_master.getAppliedOutput());
        SmartDashboard.putNumber("Right Motor Output", right_follower.getAppliedOutput());
        SmartDashboard.putNumber("Setpoint", this.setpoint);
        SmartDashboard.putNumber("Controller Output", output);
        SmartDashboard.putNumber("Feedforward Output", feedforward_output);
        SmartDashboard.putNumber("Hold Position", holdPosition);
    }

    public boolean isMovingToBounds() {
        return inBottomBounds(setpoint) && inTopBounds(setpoint);
    }

    public boolean inBottomBounds(double position) {  // Combined version
        return position >= Constants.ElevatorConstants.LOWER_LIMIT;
    }
    
    public boolean inTopBounds(double position) { // Combined version
        return position <= Constants.ElevatorConstants.UPPER_LIMIT;
    }

    public enum ElevatorState {
        Resting,
        Traveling,
        Holding,
        Manual
    }

    public enum Direction {
        Up,
        Down,
    }

   
}