package frc.robot.rebuilt.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Manages CTRE Orchestra playback through TalonFX motors during disabled mode. */
public class OrchestraManager {
  private static Orchestra orchestra;
  private static Map<String, String> musicMap = new HashMap<>();
  private static List<Integer> rioIds = Collections.emptyList();
  private static List<Integer> canivoreIds = Collections.emptyList();
  private static final List<TalonFX> motors = new ArrayList<>();

  /** Initialise the orchestra with the given motor IDs and music file map. */
  public static void init(
      List<Integer> rioMotorIds,
      List<Integer> canivoreMotorIds,
      Map<String, String> nameToDeployRelativePath) {
    orchestra = new Orchestra();
    rioIds = rioMotorIds != null ? rioMotorIds : Collections.emptyList();
    canivoreIds = canivoreMotorIds != null ? canivoreMotorIds : Collections.emptyList();
    musicMap = nameToDeployRelativePath != null ? nameToDeployRelativePath : new HashMap<>();
  }

  public static void loadMusic(String musicFileName) {
    if (orchestra == null || !musicMap.containsKey(musicFileName)) {
      return;
    }
    motors.clear();
    for (int id : rioIds) {
      TalonFX motor = new TalonFX(id);
      AudioConfigs config = new AudioConfigs().withAllowMusicDurDisable(true);
      motor.getConfigurator().apply(config, 0);
      orchestra.addInstrument(motor);
      motors.add(motor);
    }
    CANBus canivoreBus = new CANBus("canivore");
    for (int id : canivoreIds) {
      TalonFX motor = new TalonFX(id, canivoreBus);
      AudioConfigs config = new AudioConfigs().withAllowMusicDurDisable(true);
      motor.getConfigurator().apply(config, 0);
      orchestra.addInstrument(motor, 0);
      motors.add(motor);
    }
    orchestra.loadMusic(
        Filesystem.getDeployDirectory().toPath().resolve(musicMap.get(musicFileName)).toString());
  }

  public static void play() {
    if (orchestra == null) {
      return;
    }
    StatusCode status = orchestra.play();
    if (!status.isOK()) {
      System.err.println("Failed to play music: " + status);
    }
  }

  public static boolean isPlaying() {
    return orchestra != null && orchestra.isPlaying();
  }

  public static void stop() {
    if (orchestra != null) {
      orchestra.stop();
    }
  }

  public static void playTone(double frequencyHz) {
    MusicTone tone = new MusicTone(frequencyHz);
    for (TalonFX motor : motors) {
      motor.setControl(tone);
    }
  }

  public static void stopTone() {
    NeutralOut neutral = new NeutralOut();
    for (TalonFX motor : motors) {
      motor.setControl(neutral);
    }
  }
}
