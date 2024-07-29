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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;

import org.sciborgs1155.robot.Robot;

public class Wrist extends SubsystemBase {
  public static Wrist create() {
    return Robot.isReal() ? new Wrist(new RealWrist()) : new Wrist(new SimWrist());
  }

  public static WristIO none() {
    return new NoWrist();
  }

  private final WristIO hardware;

  private final ProfiledPIDController pivotFeedback;
  private final ArmFeedforward pivotFeedforward;

  private final WristVisualizer setpointVisualizer;
  private final WristVisualizer measurementVisualizer;

  public Wrist(WristIO hardware) {
    this.hardware = hardware;

    pivotFeedback =
        new ProfiledPIDController(
            Pivot.kP,
            Pivot.kI,
            Pivot.kD,
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    pivotFeedforward = new ArmFeedforward(Pivot.kS, Pivot.kG, Pivot.kV);

    setpointVisualizer = new WristVisualizer(new Color8Bit(Color.kBlue));
    measurementVisualizer = new WristVisualizer(new Color8Bit(Color.kBlue));
  }

  @Log
  public double goal() {
    return pivotFeedback.getGoal().position;
  }

  @Log
  public double setpoint() {
    return pivotFeedback.getSetpoint().position;
  }

  @Log
  public double measurement() {
    return hardware.getPosition();
  }

  public Command goTo(double angle) {
    double newAngle = MathUtil.clamp(angle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));

    return runOnce(() -> pivotFeedback.setGoal(newAngle))
        .andThen(
            run(
                () -> {
                  double prevVelocity = pivotFeedback.getSetpoint().velocity;
                  double feedback = pivotFeedback.calculate(hardware.getPosition());
                  double accel =
                      (pivotFeedback.getSetpoint().velocity - prevVelocity) / PERIOD.in(Seconds);
                  double feedforward =
                      pivotFeedforward.calculate(
                          pivotFeedback.getSetpoint().position,
                          pivotFeedback.getSetpoint().velocity,
                          accel);

                  hardware.setVoltage(feedback + feedforward);
                }));
  }

  @Override
  public void periodic() {
      if (hardware.atLimitSwitch()) {
        hardware.resetEncoder();
      }

      setpointVisualizer.setAngle(setpoint());
      measurementVisualizer.setAngle(measurement());
  }
}
