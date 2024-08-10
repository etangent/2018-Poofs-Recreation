package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Drive extends SubsystemBase implements Logged {
    public static Drive create() {
        return Robot.isReal() ? new Drive(new RealDrive()) : new Drive(new SimDrive());
    }
    
    public static Drive none() {
        return new Drive(new NoDrive());
    }

    private final DriveIO hardware;

    private final PIDController rightFeedback = new PIDController(kP, kI, kD);

    private final PIDController leftFeedback = new PIDController(kP, kI, kD);
    
    private final SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private final SimpleMotorFeedforward leftFeedforward= new SimpleMotorFeedforward(kS, kV, kA);

    public Drive(DriveIO hardware) {
        this.hardware = hardware;
    }

    public Command drive(InputStream leftVelocity, InputStream rightVelocity) {
        return run(() -> hardware.setVoltages(getVoltage(leftVelocity, false), getVoltage(rightVelocity, true)));
    }

    /**
     * @param control velocity for that side
     * @param isRight if true, controls right side; otherwise, controls the left side
     * @return voltage to set to motors
     */
    public double getVoltage(InputStream velocity, boolean isRight) {
        PIDController pid;
        SimpleMotorFeedforward ff;
        DoubleSupplier measurement;
        if (isRight) {
            pid = rightFeedback;
            ff = rightFeedforward;
            measurement = () -> hardware.getRightVelocity();
        } else {
            pid = leftFeedback;
            ff = leftFeedforward;
            measurement = () -> hardware.getLeftVelocity();
        }

        double prevVelocity = pid.getSetpoint();
        double feedbackOut = pid.calculate(measurement.getAsDouble(), velocity.getAsDouble());
        double accel = (velocity.getAsDouble() - prevVelocity) / Constants.PERIOD.in(Seconds);
        double feedforwardOut = ff.calculate(velocity.getAsDouble(), accel);

        return feedbackOut + feedforwardOut;
    }
}
