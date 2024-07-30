package org.sciborgs1155.robot.intake;

public interface IntakeIO {
  public void setRollerPower(double power);

  /**
   * @param clamp true means it clamps
   */
  public void setClamp(ClampState clamp);

  public double rollerCurrent();

  public boolean hasCube();

  public static enum ClampState {
    OPEN {
      @Override
      public boolean getLeft() {
        return true;
      }

      @Override
      public boolean getRight() {
        return true;
      }
    },
    DEFAULT {
      @Override
      public boolean getLeft() {
        return false;
      }

      @Override
      public boolean getRight() {
        return true;
      }
    },
    CLAMP {
      @Override
      public boolean getLeft() {
        return false;
      }

      @Override
      public boolean getRight() {
        return false;
      }
    };

    public abstract boolean getLeft();

    public abstract boolean getRight();
  }
}
