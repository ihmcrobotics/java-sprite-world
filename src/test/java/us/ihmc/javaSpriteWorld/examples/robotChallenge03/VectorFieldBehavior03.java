package us.ihmc.javaSpriteWorld.examples.robotChallenge03;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

public class VectorFieldBehavior03 implements Robot03Behavior
{
   private static final double targetCruisingVelocity = 2.0;
   private static final int weightExponent = 2;

   private static final double kBounds = 2.0;
   private static final double kPredator = -2.0;
   private static final double kFood = 1.0;

   private static final double kLookAhead = 0.5;

   private double xPosition, yPosition;
   private double velocity, heading;

   private ArrayList<Pair<Vector2D, Vector2D>> locationAndVelocityOfAllFood;
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators;
   private final double[] accelerationAndTurnRate = new double[2];

   @Override
   public void senseGlobalLocation(double x, double y)
   {
      this.xPosition = x;
      this.yPosition = y;
   }

   @Override
   public void senseVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   @Override
   public void senseHeading(double heading)
   {
      this.heading = heading;
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {

   }

   @Override
   public void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood)
   {
      this.locationAndVelocityOfAllFood = locationOfAllFood;
   }

   @Override
   public void sensePredators(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void senseTreasure(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllTreasure)
   {

   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      double turnRate = 0.0;
      double totalWeight = 0.0;

      Pair<Double, Double> boundsTurn = getDesiredTurnRate(5.0, 5.0, kBounds);
      double weight = Math.pow(boundsTurn.getRight(), weightExponent);
      totalWeight += weight;

      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         turnRate += getDesiredTurnRate(locationOfAllPredators.get(i).getLeft().getX(), locationAndVelocityOfAllFood.get(i).getLeft().getY(), kPredator);
      }

      for (int i = 0; i < locationAndVelocityOfAllFood.size(); i++)
      {
         turnRate += getDesiredTurnRate(locationAndVelocityOfAllFood.get(i).getLeft().getX(), locationAndVelocityOfAllFood.get(i).getLeft().getY(), kFood);
      }

      Vector2D summedAttraction = new Vector2D(getAttractionVector(5.0, 5.0, kBounds));
      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         summedAttraction.interpolate(getAttractionVector(locationOfAllPredators.get(i).getLeft().getX(),
                                                          locationOfAllPredators.get(i).getLeft().getY(), kPredator / locationOfAllPredators.size()),
                                      0.5);
      }
      for (int i = 0; i < locationAndVelocityOfAllFood.size(); i++)
      {
         summedAttraction.interpolate(getAttractionVector(locationAndVelocityOfAllFood.get(i).getLeft().getX(),
                                                          locationAndVelocityOfAllFood.get(i).getLeft().getY(), kFood / locationAndVelocityOfAllFood.size()),
                                      0.5);
      }

      double desiredHeading = EuclidCoreTools.trimAngleMinusPiToPi(Math.atan2(-robotToTarget.getX(), robotToTarget.getY()));
      double deltaHeading = EuclidCoreTools.angleDifferenceMinusPiToPi(desiredHeading, heading);
      turnRate = deltaHeading * kTurnRate;

      accelerationAndTurnRate[0] = 0.6 * (targetCruisingVelocity - velocity);
      accelerationAndTurnRate[1] = turnRate;

      return accelerationAndTurnRate;
   }

   private Vector2D getAttractionVector(double xTarget, double yTarget, double kWeight)
   {
      Vector2D robotToTarget = new Vector2D(xTarget, yTarget);
      robotToTarget.sub(xPosition, yPosition);
      double distance = EuclidCoreTools.norm(robotToTarget.getX(), robotToTarget.getY());
      robotToTarget.normalize();
      robotToTarget.scale(Math.pow(distance, weightExponent));
      return robotToTarget;
   }

   private Pair<Double, Double> getDesiredTurnRate(double xTarget, double yTarget, double kTurnRate)
   {
      Vector2D robotToTarget = new Vector2D(xTarget, yTarget);
      robotToTarget.sub(xPosition, yPosition);
      double desiredHeading = EuclidCoreTools.trimAngleMinusPiToPi(Math.atan2(-robotToTarget.getX(), robotToTarget.getY()));
      double deltaHeading = EuclidCoreTools.angleDifferenceMinusPiToPi(desiredHeading, heading);
      return Pair.of(deltaHeading * , robotToTarget.length());
   }

   private boolean checkNearBounds()
   {
      Vector2D lookAheadPosition = new Vector2D();
      lookAheadPosition.set(xPosition, yPosition);
      Vector2D xyVelocity = new Vector2D();
      xyVelocity.setX(-Math.sin(heading) * velocity);
      xyVelocity.setY(Math.cos(heading) * velocity);
      Vector2D lookAheadVector = new Vector2D(xyVelocity);
      lookAheadVector.scale(kLookAhead);
      lookAheadPosition.add(lookAheadVector);

      return lookAheadPosition.getX() < 0.0 || lookAheadPosition.getX() > 10.0 || lookAheadPosition.getY() < 0.0 || lookAheadPosition.getY() > 10.0;
   }
}
