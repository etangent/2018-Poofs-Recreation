package org.sciborgs1155.robot.wrist;

import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Wrist extends SubsystemBase implements Logged {
  public static Wrist create() {
    return Robot.isReal() ? new Wrist(new RealWrist()) : new Wrist(new SimWrist());
  }

  public static WristIO none() {
    return new NoWrist();
  }

  private final WristIO hardware;

  private final ProfiledPIDController pivotFeedback;
  private final ArmFeedforward pivotFeedforward;

  @Log.NT private final WristVisualizer setpointVisualizer = new WristVisualizer(new Color8Bit(Color.kBlue));

  @Log.NT private final WristVisualizer measurementVisualizer = new WristVisualizer(new Color8Bit(Color.kRed));

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

  public Command goTo(DoubleSupplier angle) {
    DoubleSupplier newAngle =
        () -> MathUtil.clamp(angle.getAsDouble(), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));

    return run(() -> {
          double prevVelocity = pivotFeedback.getSetpoint().velocity;
          double feedback = pivotFeedback.calculate(hardware.getPosition(), newAngle.getAsDouble());
          double accel = (pivotFeedback.getSetpoint().velocity - prevVelocity) / PERIOD.in(Seconds);
          double feedforward =
              pivotFeedforward.calculate(
                  pivotFeedback.getSetpoint().position,
                  pivotFeedback.getSetpoint().velocity,
                  accel);

          hardware.setVoltage(feedback + feedforward);
        })
        .andThen(Commands.idle(this));
  }

  @Override
  public void periodic() {
    if (hardware.atLimitSwitch()) {
      hardware.zeroEncoders();
    }

    setpointVisualizer.setAngle(setpoint());
    measurementVisualizer.setAngle(measurement());
  }
}
