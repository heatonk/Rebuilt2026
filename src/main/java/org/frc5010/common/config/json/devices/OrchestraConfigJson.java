package org.frc5010.common.config.json.devices;

import org.frc5010.common.utils.OrchestraManager;

public class OrchestraConfigJson {
  public int[] rioIds = {};

  // TalonFX CAN IDs on the CANivore bus
  public int[] canivoreIds = {};

  public static class MusicEntry {
    public String name = "";
    public String path = "";
  }

  public MusicEntry[] music = {};

  public void configure() {
    OrchestraManager.init(this);
  }
}
