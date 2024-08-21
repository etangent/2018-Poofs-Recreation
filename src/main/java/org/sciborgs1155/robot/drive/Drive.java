package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.drive.DriveConstants.*;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.TRACK_WIDTH;
import static org.sciborgs1155.robot.drive.DriveConstants.INITIAL_POSE;
import static org.sciborgs1155.robot.drive.DriveConstants.VELOCITY_TOLERANCE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Drive extends SubsystemBase implements Logged, AutoCloseable {
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

  private final SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

  private final DifferentialDriveOdometry odometry;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH);

  @Log.NT private final Field2d field2d = new Field2d();

  public Drive(DriveIO hardware) {
    this.hardware = hardware;

    odometry =
        new DifferentialDriveOdometry(
            hardware.getHeading(),
            hardware.getLeftPosition(),
            hardware.getRightPosition(),
            INITIAL_POSE);

    leftFeedback.setTolerance(VELOCITY_TOLERANCE.in(MetersPerSecond));
    rightFeedback.setTolerance(VELOCITY_TOLERANCE.in(MetersPerSecond));
  }

  @Log.NT
  public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(hardware.getHeading(), wheelPositions(), INITIAL_POSE);
  }

  @Log.NT
  public boolean atSetpoint() {
    return leftFeedback.atSetpoint() && rightFeedback.atSetpoint();
  }

  @Log.NT
  public DifferentialDriveWheelPositions wheelPositions() {
    return new DifferentialDriveWheelPositions(
        hardware.getLeftPosition(), hardware.getRightPosition());
  }

  @Log.NT
  public DifferentialDriveWheelSpeeds wheelSpeedSetpoints() {
    return new DifferentialDriveWheelSpeeds(
        leftFeedback.getSetpoint(), rightFeedback.getSetpoint());
  }

  @Log.NT
  public DifferentialDriveWheelSpeeds wheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        hardware.getLeftVelocity(), hardware.getRightVelocity());
  }

  public ChassisSpeeds chassisSpeeds() {
    return kinematics.toChassisSpeeds(wheelSpeeds());
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setWheelSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
  }

  public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    hardware.setVoltages(
        getVoltage(wheelSpeeds.leftMetersPerSecond, Side.LEFT),
        getVoltage(wheelSpeeds.rightMetersPerSecond, Side.RIGHT));
  }

  public Command drive(InputStream givenLeftVelocity, InputStream givenRightVelocity) {
    InputStream leftVelocity =
        () ->
            MathUtil.clamp(
                givenLeftVelocity.get(),
                -MAX_SPEED.in(MetersPerSecond),
                MAX_SPEED.in(MetersPerSecond));
    InputStream rightVelocity =
        () ->
            MathUtil.clamp(
                givenRightVelocity.get(),
                -MAX_SPEED.in(MetersPerSecond),
                MAX_SPEED.in(MetersPerSecond));

    return run(() ->
            setWheelSpeeds(
                new DifferentialDriveWheelSpeeds(
                    leftVelocity.getAsDouble(), rightVelocity.getAsDouble())))
        .withName("driving");
  }

  public double getVoltage(double velocity, Side side) {
    if (side == Side.LEFT) {
      return getVoltage(velocity, leftFeedback, leftFeedforward, hardware::getLeftVelocity);
    }

    return getVoltage(velocity, rightFeedback, rightFeedforward, hardware::getRightVelocity);
  }

  public double getVoltage(
      double velocity, PIDController pid, SimpleMotorFeedforward ff, DoubleSupplier measurement) {

    double prevVelocity = pid.getSetpoint();
    double feedbackOut = pid.calculate(measurement.getAsDouble(), velocity);
    double accel = (velocity - prevVelocity) / Constants.PERIOD.in(Seconds);
    double feedforwardOut = ff.calculate(velocity, accel);

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

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
