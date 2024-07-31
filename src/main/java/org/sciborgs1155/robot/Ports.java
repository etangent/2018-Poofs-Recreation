package org.sciborgs1155.robot;

public final class Ports {
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 10;
    public static final int FRONT_RIGHT_DRIVE = 12;
    public static final int REAR_RIGHT_DRIVE = 13;

    public static final int FRONT_LEFT_TURNING = 15;
    public static final int REAR_LEFT_TURNING = 14;
    public static final int FRONT_RIGHT_TURNING = 16;
    public static final int REAR_RIGHT_TURNING = 17;
  }

  public static final class Intake {
    public static final int LEFT_ROLLER = -1;
    public static final int RIGHT_ROLLER = -1;
    public static final int LEFT_SOLENOID = -1;
    public static final int RIGHT_SOLENOID = -1;
    public static final int LEFT_BEAM = -1;
    public static final int RIGHT_BEAM = -1;
  }

  public static final class Wrist {
    public static final int PIVOT = -1;
    public static final int SWITCH = -1;
  }

  public static final class Elevator {
    public static final int LEADER = -1;
    public static final int RIGHT_FOLLOWER = -1;
    public static final int LEFT_FOLLOWER_1 = -1;
    public static final int LEFT_FOLLOWER_2 = -1;
    public static final int SHIFTER = -1;
    public static final int SWITCH = -1;
  }
}
