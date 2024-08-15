package org.sciborgs1155.robot.forklift;

public interface ForkliftIO extends AutoCloseable {
  public void set(boolean extended);
}
