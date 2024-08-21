package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged, AutoCloseable {
  public static Elevator create() {
    return Robot.isReal() ? new Elevator(new RealElevator()) : new Elevator(new SimElevator());
  }

  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  private final ElevatorIO hardware;

  private final ProfiledPIDController elevatorFeedback;
  private final ElevatorFeedforward elevatorFeedforward;

  @Log.NT private final ElevatorVisualizer setPointVisualizer;

  @Log.NT private final ElevatorVisualizer measurementVisualizer;

  public Elevator(ElevatorIO hardware) {
    this.hardware = hardware;

    elevatorFeedback =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    elevatorFeedback.setTolerance(POSITION_TOLERANCE.in(Meters));

    setPointVisualizer = new ElevatorVisualizer(new Color8Bit(Color.kBlue));
    measurementVisualizer = new ElevatorVisualizer(new Color8Bit(Color.kRed));
  }

  @Log.NT
  public double goal() {
    return elevatorFeedback.getGoal().position;
  }

  @Log.NT
  public double setpoint() {
    return elevatorFeedback.getSetpoint().position;
  }

  @Log.NT
  public double measurement() {
    return hardware.getPosition();
  }

  @Log.NT
  public boolean atGoal() {
    return elevatorFeedback.atGoal();
  }

  public Command setGoal(double goal) {
    return runOnce(() -> elevatorFeedback.setGoal(goal));
  }

  @Log.NT
  public boolean atMaxHeight() {
    return atPosition(MAX_HEIGHT.in(Meters));
  }

  public boolean atPosition(double position) {
    return Math.abs(hardware.getPosition() - position) < POSITION_TOLERANCE.in(Meters);
  }

  /** pulls up onto climbing area */
  public Command pullUp() {
    return runOnce(() -> hardware.shiftGear(false)).andThen(stow()).onlyIf(this::atMaxHeight);
  }

  // i stole this but i understand it so it's okay
  public Command manualElevator(InputStream stickInput) {
    return goTo(
        stickInput
            .scale(MAX_MANUAL_VELOCITY.in(MetersPerSecond))
            // convert to position from velocity
            .scale(Constants.PERIOD.in(Seconds))
            .add(() -> elevatorFeedback.getGoal().position));
  }

  public Command stow() {
    return goTo(() -> 0).until(this::atGoal).asProxy().withName("stowing");
  }

  public Command fullExtend() {
    return goTo(() -> MAX_HEIGHT.in(Meters)).until(this::atGoal).asProxy().withName("fully-extending");
  }

  public Command goTo(DoubleSupplier position) {
    DoubleSupplier newPosition =
        () -> MathUtil.clamp(position.getAsDouble(), 0, MAX_HEIGHT.in(Meters));

    return run(
        () -> {
          double prevVelocity = elevatorFeedback.getSetpoint().velocity;
          double feedback =
              elevatorFeedback.calculate(hardware.getPosition(), newPosition.getAsDouble());
          double accel =
              (elevatorFeedback.getSetpoint().velocity - prevVelocity)
                  / Constants.PERIOD.in(Seconds);
          double feedforward =
              elevatorFeedforward.calculate(elevatorFeedback.getSetpoint().velocity, accel);

          hardware.setVoltage(feedback + feedforward);
        });
  }

  @Override
  public void periodic() {
    if (hardware.atLimitSwitch()) {
      hardware.zeroEncoders();
    }
    if (hardware.atLimitSwitch()) {
      hardware.zeroEncoders();
    }

    setPointVisualizer.setLength(elevatorFeedback.getSetpoint().position);
    measurementVisualizer.setLength(hardware.getPosition());
    setPointVisualizer.setLength(elevatorFeedback.getSetpoint().position);
    measurementVisualizer.setLength(hardware.getPosition());

    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
