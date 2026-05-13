// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.util.AllianceFlipUtil;
import java.util.Optional;

/** Add your docs here. */
public class FieldRegions {
  static double topTrenchLeftX = FieldConstants.TrenchZoneTop.nearAllianceLeftDanger.getX();
  static double topTrenchRightX = FieldConstants.TrenchZoneTop.nearAllianceRightDanger.getX();

  static double topTrenchY = FieldConstants.TrenchZoneTop.nearAllianceLeftDanger.getY();

  static double topOppTrenchLeftX = FieldConstants.TrenchZoneTop.oppAllianceLeftDanger.getX();
  static double topOppTrenchRightX = FieldConstants.TrenchZoneTop.oppAllianceRightDanger.getX();

  static double lowerTrenchLeftX = FieldConstants.TrenchZoneBottom.nearAllianceLeftDanger.getX();
  static double lowerTrenchRightX = FieldConstants.TrenchZoneBottom.nearAllianceRightDanger.getX();
  static double lowerTrenchY = FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger.getY();

  static double lowerOppTrenchLeftX = FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger.getX();
  static double lowerOppTrenchRightX =
      FieldConstants.TrenchZoneBottom.oppAllianceRightDanger.getX();

  static Translation2d allianceCornerOrigin = new Translation2d(0, 0);
  static Translation2d AllianceCornerTrench =
      new Translation2d(
          FieldConstants.TrenchZoneBottom.nearAlliance.getX()
              - 1 / 2 * FieldConstants.LeftTrench.depth,
          FieldConstants.fieldWidth);
  static Translation2d topRightMidTrenchCorner =
      new Translation2d(
          FieldConstants.TrenchZoneTop.oppAlliance.getX()
              - 1 / 2 * FieldConstants.RightTrench.depth,
          FieldConstants.fieldWidth);
  static Translation2d bottomRightMidTrenchCorner =
      new Translation2d(
          FieldConstants.TrenchZoneTop.oppAlliance.getX() - 1 / 2 * FieldConstants.LeftTrench.depth,
          0);
  static Translation2d oppTopRightOrigin =
      new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth);
  static Translation2d oppBottemRightOrigin = new Translation2d(FieldConstants.fieldLength, 0);

  static Rectangle2d allianceField = new Rectangle2d(allianceCornerOrigin, AllianceCornerTrench);
  static Rectangle2d upperMidField =
      new Rectangle2d(FieldConstants.Hub.farLeftCorner, topRightMidTrenchCorner);
  static Rectangle2d lowerMidField =
      new Rectangle2d(FieldConstants.Hub.farRightCorner, bottomRightMidTrenchCorner);
  static Rectangle2d oppUpperField =
      new Rectangle2d(FieldConstants.Hub.oppFarLeftCorner, oppTopRightOrigin);
  static Rectangle2d oppLowerField =
      new Rectangle2d(FieldConstants.Hub.oppFarRightCorner, oppBottemRightOrigin);

  public static void setupFieldRegions() {
    topTrenchLeftX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneTop.nearAllianceLeftDanger).getX();
    topTrenchRightX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneTop.nearAllianceRightDanger).getX();
    topTrenchY = AllianceFlipUtil.apply(FieldConstants.TrenchZoneTop.nearAllianceLeftDanger).getY();
    topOppTrenchLeftX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneTop.oppAllianceLeftDanger).getX();
    topOppTrenchRightX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneTop.oppAllianceRightDanger).getX();
    lowerTrenchLeftX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneBottom.nearAllianceLeftDanger).getX();
    lowerTrenchRightX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneBottom.nearAllianceRightDanger).getX();
    lowerTrenchY =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger).getY();
    lowerOppTrenchLeftX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger).getX();
    lowerOppTrenchRightX =
        AllianceFlipUtil.apply(FieldConstants.TrenchZoneBottom.oppAllianceRightDanger).getX();

    allianceCornerOrigin = AllianceFlipUtil.apply(new Translation2d(0, 0));
    AllianceCornerTrench =
        AllianceFlipUtil.apply(
            new Translation2d(
                FieldConstants.TrenchZoneBottom.nearAlliance.getX()
                    - 1 / 2 * FieldConstants.LeftTrench.depth,
                FieldConstants.fieldWidth));
    topRightMidTrenchCorner =
        AllianceFlipUtil.apply(
            new Translation2d(
                FieldConstants.TrenchZoneTop.oppAlliance.getX()
                    - 1 / 2 * FieldConstants.RightTrench.depth,
                FieldConstants.fieldWidth));
    bottomRightMidTrenchCorner =
        AllianceFlipUtil.apply(
            new Translation2d(
                FieldConstants.TrenchZoneTop.oppAlliance.getX()
                    - 1 / 2 * FieldConstants.LeftTrench.depth,
                0));
    oppTopRightOrigin =
        AllianceFlipUtil.apply(
            new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth));
    oppBottemRightOrigin = AllianceFlipUtil.apply(new Translation2d(FieldConstants.fieldLength, 0));

    allianceField = new Rectangle2d(allianceCornerOrigin, AllianceCornerTrench);
    upperMidField = new Rectangle2d(FieldConstants.Hub.farLeftCorner, topRightMidTrenchCorner);
    lowerMidField = new Rectangle2d(FieldConstants.Hub.farRightCorner, bottomRightMidTrenchCorner);
    oppUpperField = new Rectangle2d(FieldConstants.Hub.oppFarLeftCorner, oppTopRightOrigin);
    oppLowerField = new Rectangle2d(FieldConstants.Hub.oppFarRightCorner, oppBottemRightOrigin);
  }

  public static boolean isNearTrench(double currentX, double currentY) {
    boolean nearAllianceTop =
        ((currentX > topTrenchLeftX && currentX < topTrenchRightX) && currentY > topTrenchY);

    boolean nearOppAllianceTop =
        ((currentX > topOppTrenchLeftX && currentX < topOppTrenchRightX) && currentY > topTrenchY);

    boolean nearAllianceBottom =
        ((currentX > lowerTrenchLeftX && currentX < lowerTrenchRightX) && currentY < lowerTrenchY);

    boolean nearOppAllianceBottom =
        ((currentX > lowerOppTrenchLeftX && currentX < lowerOppTrenchRightX)
            && currentY < lowerTrenchY);

    return nearAllianceTop || nearOppAllianceTop || nearAllianceBottom || nearOppAllianceBottom;
  }

  static Translation2d leftAdjustment = new Translation2d(Meters.of(1), Meters.of(1.5));
  static Translation2d rightAdjustment = new Translation2d(Meters.of(1), Meters.of(-1.5));

  public static Optional<Translation2d> determineTargetPose(Pose2d currentPose) {
    Boolean inAllianceField = allianceField.contains(currentPose.getTranslation());
    Boolean inUpperMidField = upperMidField.contains(currentPose.getTranslation());
    Boolean inLowerMidField = lowerMidField.contains(currentPose.getTranslation());
    Boolean inOppUpperField = oppUpperField.contains(currentPose.getTranslation());
    Boolean inOppLowerField = oppLowerField.contains(currentPose.getTranslation());
    SmartDashboard.putBoolean("In Alliance Field", inAllianceField);
    SmartDashboard.putBoolean("In Upper Mid Field", inUpperMidField);
    SmartDashboard.putBoolean("In Lower Mid Field", inLowerMidField);
    SmartDashboard.putBoolean("In Opp Upper Field", inOppUpperField);
    SmartDashboard.putBoolean("In Opp Lower Field", inOppLowerField);
    if (inAllianceField) {
      return Optional.of(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    } else if (inUpperMidField) {
      return Optional.of(FieldConstants.Tower.leftUpright.plus(leftAdjustment));
    } else if (inLowerMidField) {
      return Optional.of(FieldConstants.Tower.rightUpright.plus(rightAdjustment));
    } else if (inOppUpperField) {
      return Optional.of(FieldConstants.Tower.leftUpright.plus(leftAdjustment));
    } else if (inOppLowerField) {
      return Optional.of(FieldConstants.Tower.rightUpright.plus(rightAdjustment));
    } else {
      return Optional.empty();
    }
  }
}
