// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.util;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

public class LedStrip extends SubsystemBase {
  public static final String ALL_LEDS = "all_leds";

  private int kLength = 0;
  private LEDPattern defaultPattern = LEDPattern.solid(Color.kGreen);

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public static class Segment {
    public LEDPattern pattern;
    public AddressableLEDBufferView view;
    public boolean active = false;
    public int order = 0;

    public static Segment create() {
      return new Segment();
    }

    public Segment setPattern(LEDPattern pattern) {
      this.pattern = pattern;
      return this;
    }

    public Segment setView(AddressableLEDBufferView view) {
      this.view = view;
      return this;
    }

    public Segment setActive(boolean active) {
      this.active = active;
      return this;
    }

    public Segment setOrder(int order) {
      this.order = order;
      return this;
    }

    public LEDPattern getPattern() {
      return pattern;
    }

    public AddressableLEDBufferView getView() {
      return view;
    }

    public boolean isActive() {
      return active;
    }

    public int getOrder() {
      return order;
    }
  }

  private static final Map<String, Segment> segments = new HashMap<>();
  private static LedStrip instance;

  public static void createInstance(int kPort, int length) {
    if (instance == null) {
      instance = new LedStrip(kPort, length);
    }
  }

  private LedStrip(int kPort, int length) {
    this.kLength = length;

    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    segments.put(
        ALL_LEDS,
        Segment.create()
            .setView(m_buffer.createView(0, kLength - 1))
            .setPattern(defaultPattern)
            .setActive(true));
    m_led.setLength(kLength);
    m_led.start();

    setDefaultCommand(runPattern().ignoringDisable(true));
  }

  public void setFullPattern(LEDPattern pattern) {
    this.defaultPattern = pattern;
  }

  public int getLength() {
    return kLength;
  }

  public Command runPattern() {
    return run(
        () ->
            segments.values().stream()
                .sorted(Comparator.comparingInt(Segment::getOrder))
                .forEach(
                    ap -> {
                      if (ap.isActive()) {
                        ap.getPattern().applyTo(ap.getView());
                      }
                    }));
  }

  @Override
  public void periodic() {
    m_led.setData(m_buffer);
  }

  public static void setSegmentActive(String name, boolean active) {
    if (segments.containsKey(name)) {
      segments.get(name).setActive(active);
    }
  }

  public static void addSegment(String name, LEDPattern pattern, int startIndex, int length) {
    if (null == instance) {
      return;
    }
    AddressableLEDBufferView view = instance.m_buffer.createView(startIndex, length);
    segments.put(name, Segment.create().setView(view).setPattern(pattern).setActive(true));
  }

  public static void addSegment(String name, int startIndex, int length) {
    if (null == instance) {
      return;
    }
    AddressableLEDBufferView view = instance.m_buffer.createView(startIndex, length);
    segments.put(name, Segment.create().setView(view).setPattern(LEDPattern.kOff).setActive(false));
  }

  public static void addSegment(String name, int startIndex, int length, int order) {
    if (null == instance) {
      return;
    }
    AddressableLEDBufferView view = instance.m_buffer.createView(startIndex, length);
    segments.put(
        name,
        Segment.create()
            .setView(view)
            .setPattern(LEDPattern.kOff)
            .setActive(false)
            .setOrder(order));
  }

  public static void removeSegment(String name) {
    segments.remove(name);
  }

  public static void clearSegments() {
    segments.clear();
  }

  public static void setCommand(Command defaultCommand) {
    if (null != instance) {
      instance.setDefaultCommand(defaultCommand);
    }
  }

  public static void changeSegmentPattern(String name, LEDPattern pattern) {
    if (segments.containsKey(name)) {
      segments.get(name).setPattern(pattern);
    }
  }

  public static LEDPattern getRainbowPattern(double percentScrollingSpeed) {
    return LEDPattern.rainbow(255, 255)
        .scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
  }

  public static LEDPattern getSolidPattern(Color color) {
    return LEDPattern.solid(color);
  }

  public static LEDPattern getMaskedPattern(
      LEDPattern basePattern, double percentVisible, double percentScrollingSpeed) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, percentVisible, Color.kBlack);
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
    return basePattern.mask(mask);
  }

  public static LEDPattern getBlinkingPattern(LEDPattern basePattern, Time blinkInterval) {
    return basePattern.blink(blinkInterval);
  }
}
