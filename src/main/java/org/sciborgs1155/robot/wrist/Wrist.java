package org.sciborgs1155.robot.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.wrist.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.hanger.Hanger;

public class Wrist extends SubsystemBase implements Logged, AutoCloseable {
  public static Wrist create() {
    return Robot.isReal() ? new Wrist(new RealWrist()) : new Wrist(new SimWrist());
  }

  public static WristIO none() {
    return new NoWrist();
  }

  private final WristIO hardware;

  private final ProfiledPIDController pivotFeedback;
  private final ArmFeedforward pivotFeedforward;

  @Log.NT
  private final WristVisualizer setpointVisualizer =
      new WristVisualizer(new Color8Bit(Color.kBlue));

  @Log.NT
  private final WristVisualizer measurementVisualizer =
      new WristVisualizer(new Color8Bit(Color.kRed));

  public Wrist(WristIO hardware) {
    this.hardware = hardware;

    pivotFeedback =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    pivotFeedforward = new ArmFeedforward(kS, kG, kV);

    pivotFeedback.setTolerance(POSITION_TOLERANCE.in(Radians));
  }

  @Log.NT
  public double goal() {
    return pivotFeedback.getGoal().position;
  }

  @Log.NT
  public double setpoint() {
    return pivotFeedback.getSetpoint().position;
  }

  @Log.NT
  public double measurement() {
    return hardware.getPosition();
  }

  @Log.NT
  public boolean atGoal() {
    return pivotFeedback.atGoal();
  }

  public Command setGoal(double goal) {
    return runOnce(() -> pivotFeedback.setGoal(goal));
  }

  public boolean atPosition(double position) {
    return Math.abs(hardware.getPosition() - position) < POSITION_TOLERANCE.in(Radians);
  }

  public Command stow() {
    return goTo(() -> MIN_ANGLE.in(Radians)).until(this::atGoal).asProxy().withName("stowing");
  }

  // im very proud of this name
  public Command unStow() {
    return goTo(() -> MAX_ANGLE.in(Radians)).until(this::atGoal).asProxy().withName("un-stowing");
  }

  public Command shootAngle() {
    return goTo(() -> Math.PI / 4).until(this::atGoal).asProxy();
  }

  public Command toggle() {
    return new DeferredCommand(() -> pivotFeedback.getGoal().position == 0 ? unStow() : stow(), Set.of(this));
  }

  public Command goTo(DoubleSupplier angle) {
    return run(
        () -> {
          double newAngle = MathUtil.clamp(angle.getAsDouble(), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
          double prevVelocity = pivotFeedback.getSetpoint().velocity;
          double feedback = pivotFeedback.calculate(hardware.getPosition(), newAngle);
          double accel = (pivotFeedback.getSetpoint().velocity - prevVelocity) / PERIOD.in(Seconds);
          double feedforward =
              pivotFeedforward.calculate(
                  pivotFeedback.getSetpoint().position,
                  pivotFeedback.getSetpoint().velocity,
                  accel);

          hardware.setVoltage(feedback + feedforward);
        });
  }

  public Command hold() {
    return goTo(() -> goal()).withName("holding");
  }
 
  @Override
  public void periodic() {
    if (hardware.atLimitSwitch()) {
      hardware.zeroEncoders();
    }

    setpointVisualizer.setAngle(setpoint());
    measurementVisualizer.setAngle(measurement());

    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
