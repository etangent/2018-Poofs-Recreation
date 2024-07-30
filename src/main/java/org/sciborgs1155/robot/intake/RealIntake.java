package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Ports.Intake.*;
import static org.sciborgs1155.robot.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class RealIntake implements IntakeIO {
  private final TalonFX rightRoller;
  private final TalonFX leftRoller;

  private final Solenoid leftSolenoid;
  private final Solenoid rightSolenoid;

  private final DigitalInput leftBeamBreak;
  private final DigitalInput rightBeamBreak;

  public RealIntake() {
    // roller configs
    rightRoller = new TalonFX(RIGHT_ROLLER);
    leftRoller = new TalonFX(LEFT_ROLLER);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Rollers.CURRENT_LIMIT;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rightRoller.getConfigurator().apply(rollerConfig);
    leftRoller.getConfigurator().apply(rollerConfig);

    rightRoller.setSafetyEnabled(true);
    leftRoller.setSafetyEnabled(true);

    leftRoller.setControl(new Follower(RIGHT_ROLLER, true));

    // solenoid configs
    leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, LEFT_SOLENOID);
    rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RIGHT_SOLENOID);

    // Beambreaks
    leftBeamBreak = new DigitalInput(LEFT_BEAM);
    rightBeamBreak = new DigitalInput(RIGHT_BEAM);
  }

  @Override
  public void setRollerPower(double power) {
    rightRoller.set(power);
  }

  @Override
  public void setClamp(ClampState clamp) {
    leftSolenoid.set(clamp.getLeft());
    rightSolenoid.set(clamp.getRight());
  }

  @Override
  public double rollerCurrent() {
    return rightRoller.getSupplyCurrent().getValueAsDouble()
        + leftRoller.getSupplyCurrent().getValueAsDouble();
  }

  // they did some cursed things with these two beambreaks and I'm going to ignore it
  @Override
  public boolean hasCube() {
    return leftBeamBreak.get() && rightBeamBreak.get();
  }
}
