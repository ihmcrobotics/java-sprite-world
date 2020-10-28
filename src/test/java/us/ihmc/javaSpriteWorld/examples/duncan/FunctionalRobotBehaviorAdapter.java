package us.ihmc.javaSpriteWorld.examples.duncan;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;

import java.util.ArrayList;
import java.util.function.*;

public class FunctionalRobotBehaviorAdapter implements Robot06Behavior
{
   private Consumer<Point2D> senseGlobalPositionForTestingOnlyConsumer;
   private DoubleConsumer senseNoiseFreeHeadingForTestingOnly;
   private DoubleConsumer senseVelocity;
   private DoubleConsumer senseHeading;
   private Consumer<Point2D> senseMousePressed;
   private Consumer<String> senseKeyPressed;
   private Consumer<ArrayList<Pair<Vector2D, Double>>> senseWallRangeInBodyFrame;
   private Consumer<ArrayList<Triple<Integer, Point2D, Vector2D>>> senseFoodInBodyFrame;
   private Consumer<ArrayList<Pair<Point2D, Vector2D>>> sensePredatorsInBodyFrame;
   private Consumer<Pair<Point2D, Integer>> senseClosestFlagInBodyFrame;
   private IntConsumer senseDroppedFlag;
   private IntConsumer sensePickedUpFlag;
   private IntConsumer senseDeliveredFlag;
   private IntConsumer senseCarryingFlag;
   private Runnable senseHitWall;
   private Supplier<Pair<Double, Double>> getAccelerationAndTurnRate;
   private BooleanSupplier getDropFlag;
   private Consumer<Triple<Double, Double, Double>> senseScoreHealthTime;
   private Runnable reset;
   private Runnable exit;

   public void setSenseGlobalPositionForTestingOnlyConsumer(Consumer<Point2D> senseGlobalPositionForTestingOnlyConsumer)
   {
      this.senseGlobalPositionForTestingOnlyConsumer = senseGlobalPositionForTestingOnlyConsumer;
   }

   public void setSenseNoiseFreeHeadingForTestingOnly(DoubleConsumer senseNoiseFreeHeadingForTestingOnly)
   {
      this.senseNoiseFreeHeadingForTestingOnly = senseNoiseFreeHeadingForTestingOnly;
   }

   public void setSenseVelocity(DoubleConsumer senseVelocity)
   {
      this.senseVelocity = senseVelocity;
   }

   public void setSenseHeading(DoubleConsumer senseHeading)
   {
      this.senseHeading = senseHeading;
   }

   public void setSenseMousePressed(Consumer<Point2D> senseMousePressed)
   {
      this.senseMousePressed = senseMousePressed;
   }

   public void setSenseKeyPressed(Consumer<String> senseKeyPressed)
   {
      this.senseKeyPressed = senseKeyPressed;
   }

   public void setSenseWallRangeInBodyFrame(Consumer<ArrayList<Pair<Vector2D, Double>>> senseWallRangeInBodyFrame)
   {
      this.senseWallRangeInBodyFrame = senseWallRangeInBodyFrame;
   }

   public void setSenseFoodInBodyFrame(Consumer<ArrayList<Triple<Integer, Point2D, Vector2D>>> senseFoodInBodyFrame)
   {
      this.senseFoodInBodyFrame = senseFoodInBodyFrame;
   }

   public void setSensePredatorsInBodyFrame(Consumer<ArrayList<Pair<Point2D, Vector2D>>> sensePredatorsInBodyFrame)
   {
      this.sensePredatorsInBodyFrame = sensePredatorsInBodyFrame;
   }

   public void setSenseClosestFlagInBodyFrame(Consumer<Pair<Point2D, Integer>> senseClosestFlagInBodyFrame)
   {
      this.senseClosestFlagInBodyFrame = senseClosestFlagInBodyFrame;
   }

   public void setSenseDroppedFlag(IntConsumer senseDroppedFlag)
   {
      this.senseDroppedFlag = senseDroppedFlag;
   }

   public void setSensePickedUpFlag(IntConsumer sensePickedUpFlag)
   {
      this.sensePickedUpFlag = sensePickedUpFlag;
   }

   public void setSenseDeliveredFlag(IntConsumer senseDeliveredFlag)
   {
      this.senseDeliveredFlag = senseDeliveredFlag;
   }

   public void setSenseCarryingFlag(IntConsumer senseCarryingFlag)
   {
      this.senseCarryingFlag = senseCarryingFlag;
   }

   public void setSenseHitWall(Runnable senseHitWall)
   {
      this.senseHitWall = senseHitWall;
   }

   public void setGetAccelerationAndTurnRate(Supplier<Pair<Double, Double>> getAccelerationAndTurnRate)
   {
      this.getAccelerationAndTurnRate = getAccelerationAndTurnRate;
   }

   public void setGetDropFlag(BooleanSupplier getDropFlag)
   {
      this.getDropFlag = getDropFlag;
   }

   public void setSenseScoreHealthTime(Consumer<Triple<Double, Double, Double>> senseScoreHealthTime)
   {
      this.senseScoreHealthTime = senseScoreHealthTime;
   }

   public void setReset(Runnable reset)
   {
      this.reset = reset;
   }

   public void setExit(Runnable exit)
   {
      this.exit = exit;
   }

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      if (senseGlobalPositionForTestingOnlyConsumer != null)
         senseGlobalPositionForTestingOnlyConsumer.accept(new Point2D(x, y));
   }

   @Override
   public void senseNoiseFreeHeadingForTestingOnly(double heading)
   {
      if (senseNoiseFreeHeadingForTestingOnly != null)
         senseNoiseFreeHeadingForTestingOnly.accept(heading);
   }

   @Override
   public void senseVelocity(double velocity)
   {
      if (senseVelocity != null)
         senseVelocity.accept(velocity);
   }

   @Override
   public void senseHeading(double heading)
   {
      if (senseHeading != null)
         senseHeading.accept(heading);
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      if (senseMousePressed != null)
         senseMousePressed.accept(new Point2D(mousePressedX, mousePressedY));
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {
      if (senseKeyPressed != null)
         senseKeyPressed.accept(keyPressed);
   }

   @Override
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      if (senseWallRangeInBodyFrame != null)
         senseWallRangeInBodyFrame.accept(vectorsAndDistancesToWallInBodyFrame);
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      if (senseFoodInBodyFrame != null)
         senseFoodInBodyFrame.accept(locationOfAllFood);
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      if (sensePredatorsInBodyFrame != null)
         sensePredatorsInBodyFrame.accept(locationOfAllPredators);
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      if (senseClosestFlagInBodyFrame != null)
         senseClosestFlagInBodyFrame.accept(positionInBodyFrameAndIdOfClosestFlag);
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      if (senseDroppedFlag != null)
         senseDroppedFlag.accept(flagId);
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      if (sensePickedUpFlag != null)
         sensePickedUpFlag.accept(id);
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      if (senseDeliveredFlag != null)
         senseDeliveredFlag.accept(flagId);
   }

   @Override
   public void senseCarryingFlag(int flagId)
   {
      if (senseCarryingFlag != null)
         senseCarryingFlag.accept(flagId);
   }

   @Override
   public void senseHitWall()
   {
      if (senseHitWall != null)
         senseHitWall.run();
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      if (getAccelerationAndTurnRate != null)
      {
         Pair<Double, Double> accelerationAndTurnRate = getAccelerationAndTurnRate.get();
         return new double[] {accelerationAndTurnRate.getLeft(), accelerationAndTurnRate.getRight()};
      }
      else
      {
         return new double[0];
      }
   }

   @Override
   public boolean getDropFlag()
   {
      if (getDropFlag != null)
         return getDropFlag.getAsBoolean();
      else
         return false;
   }

   @Override
   public void senseScoreHealthTime(double score, double health, double elapsedTime)
   {
      if (senseScoreHealthTime != null)
         senseScoreHealthTime.accept(Triple.of(score, health, elapsedTime));
   }

   @Override
   public void reset()
   {
      if (reset != null)
         reset.run();
   }

   @Override
   public void exit()
   {
      if (exit != null)
         exit.run();
   }
}
