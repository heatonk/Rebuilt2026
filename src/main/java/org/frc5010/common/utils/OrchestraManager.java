package org.frc5010.common.utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.HashMap;
import java.util.Map;
import org.frc5010.common.config.json.devices.OrchestraConfigJson;

/** Manages CTRE Orchestra playback through all robot TalonFX motors during disabled mode. */
public class OrchestraManager {
  private static Orchestra orchestra;
  private static Map<String, String> musicMap;
  private static OrchestraConfigJson musicConfigJson;

  public static void init(OrchestraConfigJson configJson) {
    orchestra = new Orchestra();
    musicMap = new HashMap<>();
    musicConfigJson = configJson;
    for (OrchestraConfigJson.MusicEntry entry : musicConfigJson.music) {
      musicMap.put(entry.name, entry.path);
    }
  }

  public static void loadMusic(String musicFileName) {
    if (null != orchestra) {
      for (int id : musicConfigJson.rioIds) {
        TalonFX motor = new TalonFX(id);
        AudioConfigs config = new AudioConfigs().withAllowMusicDurDisable(true);
        motor.getConfigurator().apply(config, 0);
        orchestra.addInstrument(motor);
      }
      CANBus canivoreBus = new CANBus("canivore");
      for (int id : musicConfigJson.canivoreIds) {
        TalonFX motor = new TalonFX(id, canivoreBus);
        AudioConfigs config = new AudioConfigs().withAllowMusicDurDisable(true);
        motor.getConfigurator().apply(config, 0);
        orchestra.addInstrument(motor, 0);
      }
      orchestra.loadMusic(
          Filesystem.getDeployDirectory().toPath().resolve(musicMap.get(musicFileName)).toString());
    }
  }

  /** Start playing the mariachi song. Safe to call repeatedly. */
  public static void play() {
    if (null != orchestra) {
      StatusCode status = orchestra.play();
      if (!status.isOK()) {
        System.err.println("Failed to play music: " + status);
      }
    }
  }

  public static boolean isPlaying() {
    return null != orchestra && orchestra.isPlaying();
  }

  /** Stop playback and release motors back to normal control. */
  public static void stop() {
    if (null != orchestra) {
      orchestra.stop();
    }
  }
}
