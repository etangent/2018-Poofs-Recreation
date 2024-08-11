package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.drive.DriveConstants.INITIAL_POSE;
import static org.sciborgs1155.robot.drive.DriveConstants.VELOCITY_TOLERANCE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    private final DifferentialDriveOdometry odometry;

    @Log.NT private final Field2d field2d = new Field2d();

    public Drive(DriveIO hardware) {
        this.hardware = hardware;

        odometry = new DifferentialDriveOdometry(hardware.getHeading(), hardware.getLeftPosition(), hardware.getRightPosition(), INITIAL_POSE);

        leftFeedback.setTolerance(VELOCITY_TOLERANCE.in(MetersPerSecond));
        rightFeedback.setTolerance(VELOCITY_TOLERANCE.in(MetersPerSecond));
    }

    @Log.NT
    public Pose2d pose() {
        return odometry.getPoseMeters();
    }

    @Log.NT
    public boolean atSetpoint() {
        return leftFeedback.atSetpoint() && rightFeedback.atSetpoint();
    }

    @Log.NT
    public double leftVelocitySetpoint() {
        return leftFeedback.getSetpoint();
    }

    @Log.NT
    public double leftVelocityMeasurement() {
        return hardware.getLeftVelocity();
    }

    @Log.NT
    public double rightVelocitySetpoint() {
        return rightFeedback.getSetpoint();
    }

    @Log.NT
    public double rightVelocityMeasurement() {
        return hardware.getRightVelocity();
    }

    public Command drive(InputStream leftVelocity, InputStream rightVelocity) {
        return run(() -> {
            System.out.println(rightVelocity.getAsDouble());
            hardware.setVoltages(getVoltage(leftVelocity, Side.LEFT), getVoltage(rightVelocity, Side.RIGHT));
        }).withName("driving");
    }

    public double getVoltage(InputStream velocity, Side side) {
        if (side == Side.LEFT) {
            return getVoltage(velocity, leftFeedback, leftFeedforward, hardware::getLeftVelocity);
        }

        return getVoltage(velocity, rightFeedback, rightFeedforward, hardware::getRightVelocity);
    }

    /**
     * @param control velocity for that side
     * @param isRight if true, controls right side; otherwise, controls the left side
     * @return voltage to set to motors
     */
    public double getVoltage(InputStream velocity, PIDController pid, SimpleMotorFeedforward ff, DoubleSupplier measurement) {

        double prevVelocity = pid.getSetpoint();
        double feedbackOut = pid.calculate(measurement.getAsDouble(), velocity.getAsDouble());
        double accel = (velocity.getAsDouble() - prevVelocity) / Constants.PERIOD.in(Seconds);
        double feedforwardOut = ff.calculate(velocity.getAsDouble(), accel);

        return feedbackOut + feedforwardOut;
    }

    @Override
    public void periodic() {
        odometry.update(hardware.getHeading(), hardware.getLeftPosition(), hardware.getRightPosition());
        field2d.setRobotPose(pose());
        log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
    }

    public enum Side {
        LEFT,
        RIGHT
    }
}
