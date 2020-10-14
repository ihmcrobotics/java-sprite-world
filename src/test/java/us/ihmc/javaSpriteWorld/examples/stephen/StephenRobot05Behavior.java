package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import org.junit.jupiter.api.Assertions;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.*;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class StephenRobot05Behavior implements Robot05Behavior, Robot06Behavior
{
   // simulator parameters
   private final double initialX = 0.5;
   private final double initialY = 0.5;
   private final double dt = 0.01;
   private final List<Point2D> wallPoints = new ArrayList<>();

   // robot state
   private double velocity;
   private double heading;
   private final Point2D xyPosition = new Point2D(initialX, initialY);
   private final Vector2D xyVelocity = new Vector2D();
   private final Vector2D xyHeading = new Vector2D();

   // environment state
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFoodInBodyFrame;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators;
   private Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag;
   private ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame;

   // behavior parameters
   private final Vector2D wallForce = new Vector2D();
   private final Vector2D foodForce = new Vector2D();
   private final Vector2D predatorForce = new Vector2D();
   private final Vector2D flagForce = new Vector2D();

   private final Vector2D totalForce = new Vector2D();
   private boolean firstTick = true;
   private final double[] previousAction = new double[2];
   private final double[] totalAction = new double[2];
   private final double[] filteredAction = new double[2];

   private final double maxForceWall = 0.4;
   private final double maxForceFood = 1.25;
   private final double maxForcePredator = 1.2;
   private final double maxForceAvoidFlagWhileDelivering = 3.0;
   private final double flagAttractionForceMagnitude = 0.5;
   private final double exploreAreaForceMagnitude = 0.4;

   private final double minAngleToPenalizeFood = Math.toRadians(60.0);
   private final double extraFoodDistanceIfBehind = 1.0;
   private final double proximityNearWallToIgnoreFood = 0.8;

   private final Point2D[] flagLocations = new Point2D[5];

   private final Point2D[] areasToExplore;
   private int counter = 0;
   private final int counterToSwitchExploreArea = 500;
   private final double proximityToExploreAreaToSwitch = 0.4;
   private final Point2D areaToExplore = new Point2D();
   private final Random random = new Random(3920);

   private final double baseWall = 2.75;
   private double baseFood = 2.2;
   private final double basePredator = 2.0;
   private final double baseFlag = 2.5;

   // toggle sub-behaviors
   private boolean enableWall = true;
   private boolean enableFood = true;
   private boolean enablePredators = true;
   private boolean enableFlags = false;

   private int flagIdToChase = 1;
   private boolean inDeliverFlagMode = false;

   // filter parameters
   private final double alphaAction = 1.0;
   private final double alphaVelocity = 1.0;
   private final double alphaRangeSensor = 0.33; // not an alpha filter, just scales down correction

   public StephenRobot05Behavior()
   {
      for (int i = 0; i < flagLocations.length; i++)
      {
         flagLocations[i] = new Point2D();
         flagLocations[i].setToNaN();
      }

      areasToExplore = new Point2D[9];
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            areasToExplore[3 * i + j] = new Point2D(5 + 3 * (i - 1), 5 + 3 * (j - 1));
         }
      }

      switchAreaToExplore();

      wallPoints.add(new Point2D(0.0, 0.0));
      wallPoints.add(new Point2D(0.0, 10.0));
      wallPoints.add(new Point2D(10.0, 10.0));
      wallPoints.add(new Point2D(10.0, 0.0));
   }

   @Override
   public void senseVelocity(double velocity)
   {
      this.velocity = filter(alphaVelocity, velocity, this.velocity);
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
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      this.vectorsAndDistancesToWallInBodyFrame = vectorsAndDistancesToWallInBodyFrame;
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFoodInBodyFrame = locationOfAllFood;
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      this.positionInBodyFrameAndIdOfClosestFlag = positionInBodyFrameAndIdOfClosestFlag;
   }

   private void calculateWorldFrameCoordinates()
   {
      xyHeading.setX(-Math.sin(heading));
      xyHeading.setY(Math.cos(heading));

      xyVelocity.set(xyHeading);
      xyVelocity.scale(velocity);

      xyPosition.addX(xyVelocity.getX() * dt);
      xyPosition.addY(xyVelocity.getY() * dt);

      doLocalization();
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      calculateWorldFrameCoordinates();
      updateAreaToExplore();
      takeANoteOfFlagLocation();

      wallForce.setToZero();
      foodForce.setToZero();
      predatorForce.setToZero();
      flagForce.setToZero();

      // wall
      computeWallForce();

      // food
      for (int i = 0; i < locationOfAllFoodInBodyFrame.size(); i++)
      {
         computeFoodForce(foodForce, locationOfAllFoodInBodyFrame.get(i).getMiddle());
      }

      // predators
      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         computePredatorForce(predatorForce, locationOfAllPredators.get(i).getLeft());
      }

      // chase flag
      if (inDeliverFlagMode)
      {
         Point2D dropPointWorld = new Point2D(9.0, 9.0);
         Point2D dropPointBody = new Point2D();
         worldFrameToBodyFrame(dropPointWorld, dropPointBody);

         computeGoToFlagForce(flagForce, dropPointBody);
         computeAvoidFlagForce(flagForce, positionInBodyFrameAndIdOfClosestFlag.getLeft());
      }
      else
      {
         boolean detectedFlagShouldBeRetrieved = positionInBodyFrameAndIdOfClosestFlag.getRight() == flagIdToChase;
         if (detectedFlagShouldBeRetrieved)
         {
            computeGoToFlagForce(flagForce, positionInBodyFrameAndIdOfClosestFlag.getLeft());
         }
         else
         {
            if (flagLocations[flagIdToChase - 1].containsNaN())
            {
               computeGoToWorldCoordinate(flagForce, areaToExplore, exploreAreaForceMagnitude);
            }
            else
            {
               computeGoToWorldCoordinate(flagForce, flagLocations[flagIdToChase - 1], flagAttractionForceMagnitude);
            }
         }
      }

      toggleOffSubBehaviors();
      totalForce.setX(wallForce.getX() + predatorForce.getX() + foodForce.getX() + flagForce.getX());
      totalForce.setY(wallForce.getY() + predatorForce.getY() + foodForce.getY() + flagForce.getY());

      double totalForceMagnitude = totalForce.length();
      double percentageWall = wallForce.length() / totalForceMagnitude;
      double percentageFood = foodForce.length() / totalForceMagnitude;
      double percentagePredator = predatorForce.length() / totalForceMagnitude;
      double percentageFlagAttractor = flagForce.length() / totalForceMagnitude;
      double percentageFlagRepulsive = flagForce.length() / totalForceMagnitude;

      // convert to acceleration and steering action
      double deltaDesiredHeading = headingFromVector(totalForce.getX(), totalForce.getY());

      double velocityWhenAligned = 3.0;
      double targetVelocity;
      double angleToStopAndTurn = Math.toRadians(60.0);

      if (Math.abs(deltaDesiredHeading) < angleToStopAndTurn)
      {
         targetVelocity = velocityWhenAligned - (velocityWhenAligned / angleToStopAndTurn) * Math.abs(deltaDesiredHeading);
      }
      else
      {
         targetVelocity = 0.0;
      }

      double kAcceleration = 1.5;
      double accelerationAction = kAcceleration * (targetVelocity - velocity);

      double kTurn = 2.0;
      double turningAction = kTurn * deltaDesiredHeading;

      totalAction[0] = accelerationAction;
      totalAction[1] = turningAction;

      if (firstTick)
      {
         firstTick = false;
         for (int i = 0; i < 2; i++)
         {
            filteredAction[i] = totalAction[i];
         }
      }
      else
      {
         for (int i = 0; i < 2; i++)
         {
            filteredAction[i] = filter(alphaAction, totalAction[i], previousAction[i]);
         }
      }

      for (int i = 0; i < 2; i++)
      {
         previousAction[i] = totalAction[i];
      }

      return filteredAction;
   }

   private void doLocalization()
   {
      int numberOfCorrections = 0;
      Vector2D totalCorrectionVector = new Vector2D();

      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
      {
         Pair<Vector2D, Double> sensorReading = vectorsAndDistancesToWallInBodyFrame.get(i);

         Point2D wallPointInBody = new Point2D(sensorReading.getLeft());
         wallPointInBody.scale(1.0 / wallPointInBody.distanceFromOrigin());
         wallPointInBody.scale(sensorReading.getRight());

         Point2D wallPointInWorldEstimatedFrame = new Point2D();
         bodyFrameToWorldFrame(wallPointInBody, wallPointInWorldEstimatedFrame);

         Vector2D sensorDirectionInBody = new Vector2D(sensorReading.getLeft());
         Vector2D sensorDirectionInWorld = new Vector2D();
         bodyFrameToWorldFrame(sensorDirectionInBody, sensorDirectionInWorld);

         for (int j = 0; j < wallPoints.size(); j++)
         {
            Point2D expectedIntersectionPoint = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(xyPosition,
                                                                                                             sensorDirectionInWorld,
                                                                                                             wallPoints.get(j),
                                                                                                             wallPoints.get((j + 1) % wallPoints.size()));
            if (expectedIntersectionPoint != null)
            {
               numberOfCorrections++;

               Vector2D correctionVector = new Vector2D(expectedIntersectionPoint);
               correctionVector.sub(wallPointInWorldEstimatedFrame);
               totalCorrectionVector.add(correctionVector);

               break;
            }
         }
      }

      if (numberOfCorrections > 0)
      {
         totalCorrectionVector.scale(alphaRangeSensor / numberOfCorrections);
         xyPosition.add(totalCorrectionVector);
      }
   }

   private void toggleOffSubBehaviors()
   {
      if (!enableWall)
      {
         wallForce.setToZero();
      }
      if (!enableFood)
      {
         foodForce.setToZero();
      }
      if (!enablePredators)
      {
         predatorForce.setToZero();
      }
      if (!enableFlags)
      {
         flagForce.setToZero();
      }
   }

   private void updateAreaToExplore()
   {
      if (counter++ >= counterToSwitchExploreArea)
      {
         switchAreaToExplore();
         counter = 0;
      }
      else if (areaToExplore.distance(xyPosition) < proximityToExploreAreaToSwitch)
      {
         switchAreaToExplore();
         counter = 0;
      }
   }

   private void switchAreaToExplore()
   {
      while (true)
      {
         Point2D newAreaToExplore = areasToExplore[random.nextInt(areasToExplore.length)];
         if (newAreaToExplore.equals(areaToExplore))
         {
            continue;
         }
         else
         {
            areaToExplore.set(newAreaToExplore);
            return;
         }
      }
   }

   private void takeANoteOfFlagLocation()
   {
      int flagZeroIndexId = positionInBodyFrameAndIdOfClosestFlag.getRight() - 1;
      if (flagLocations[flagZeroIndexId].containsNaN())
      {
         bodyFrameToWorldFrame(positionInBodyFrameAndIdOfClosestFlag.getLeft(), flagLocations[flagZeroIndexId]);
      }
   }

   private void computeWallForce()
   {
      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
      {
         Pair<Vector2D, Double> sensorData = vectorsAndDistancesToWallInBodyFrame.get(i);
         Vector2D sensedWallForce = new Vector2D(sensorData.getLeft());
         sensedWallForce.normalize();
         sensedWallForce.scale(-1.0);
         sensedWallForce.scale(maxForceWall * Math.pow(baseWall, - sensorData.getRight()));

         wallForce.add(sensedWallForce);
      }
   }

   private void computeFoodForce(Vector2D forceToAddTo, Point2D foodInBodyFrame)
   {
      Vector2D force = new Vector2D();

      // Dead reckoning seems off...
      double distanceToWall = computeDistanceToWall(new Point2D(foodInBodyFrame));
      if (distanceToWall < proximityNearWallToIgnoreFood)
      {
         return;
      }

      force.set(foodInBodyFrame);
      force.normalize();

      double distance = EuclidCoreTools.norm(foodInBodyFrame.getX(), foodInBodyFrame.getY());
      double angleFromStraightAhead = headingFromVector(foodInBodyFrame);
      if (Math.abs(angleFromStraightAhead) > minAngleToPenalizeFood)
      {
         double angleFromStraightBehind = EuclidCoreTools.angleDifferenceMinusPiToPi(angleFromStraightAhead, Math.PI);
         distance += extraFoodDistanceIfBehind * (1.0 - Math.abs(angleFromStraightBehind) / (Math.PI - minAngleToPenalizeFood));
      }

      force.scale(maxForceFood * Math.pow(baseFood, - distance));
      forceToAddTo.add(force);
   }

   private void computePredatorForce(Vector2D forceToAddTo, Tuple2DReadOnly predatorInBodyFrame)
   {
      Vector2D force = new Vector2D();

      force.set(predatorInBodyFrame);
      force.negate();
      force.normalize();

      double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
      force.scale(maxForcePredator * Math.pow(basePredator, - distance));
      forceToAddTo.add(force);
   }

   private void computeAvoidFlagForce(Vector2D forceToAddTo, Tuple2DReadOnly flagInBodyFrame)
   {
      Vector2D force = new Vector2D();

      double fieldOfViewForAvoiding = Math.toRadians(45.0);
      double angle = headingFromVector(flagInBodyFrame);
      if (Math.abs(angle) > fieldOfViewForAvoiding)
      {
         return;
      }

      force.set(flagInBodyFrame);
      force.negate();
      force.normalize();

      double distance = EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY());
      force.scale(maxForceAvoidFlagWhileDelivering * Math.pow(baseFlag, - distance));
      forceToAddTo.add(force);
   }

   private void computeGoToFlagForce(Vector2D forceToAddTo, Tuple2DReadOnly flagInBodyFrame)
   {
      Vector2D force = new Vector2D();

      force.set(flagInBodyFrame);
      force.normalize();

      force.scale(flagAttractionForceMagnitude);
      forceToAddTo.add(force);
   }

   private void computeGoToWorldCoordinate(Vector2D forceToAddTo, Point2DReadOnly worldCoordinate, double forceMagnitude)
   {
      Point2D bodyFrameCoordinate = new Point2D();
      worldFrameToBodyFrame(worldCoordinate, bodyFrameCoordinate);

      Vector2D force = new Vector2D(bodyFrameCoordinate);
      force.normalize();
      force.scale(forceMagnitude);

      forceToAddTo.add(force);
   }

   private double computeDistanceToWall(Point2DReadOnly foodInBodyFrame)
   {
      Point2D foodInWorld = new Point2D();
      bodyFrameToWorldFrame(foodInBodyFrame, foodInWorld);
      return Math.min(Math.min(foodInWorld.getX(), foodInWorld.getY()), Math.min(10.0 - foodInWorld.getX(), 10.0 - foodInWorld.getY()));
   }

   private static double headingFromVector(Tuple2DReadOnly vector)
   {
      return headingFromVector(vector.getX(), vector.getY());
   }

   private static double headingFromVector(double x, double y)
   {
      return Math.atan2(-x, y);
   }

   private void bodyFrameToWorldFrame(Point2DReadOnly bodyFramePoint, Point2DBasics worldFramePointToSet)
   {
      double relativeWorldVectorX = bodyFramePoint.getX() * Math.cos(heading) - bodyFramePoint.getY() * Math.sin(heading);
      double relativeWorldVectorY = bodyFramePoint.getX() * Math.sin(heading) + bodyFramePoint.getY() * Math.cos(heading);

      worldFramePointToSet.setX(relativeWorldVectorX + xyPosition.getX());
      worldFramePointToSet.setY(relativeWorldVectorY + xyPosition.getY());
   }

   private void worldFrameToBodyFrame(Point2DReadOnly worldFramePoint, Point2DBasics bodyFramePointToSet)
   {
      double dx = worldFramePoint.getX() - xyPosition.getX();
      double dy = worldFramePoint.getY() - xyPosition.getY();

      bodyFramePointToSet.setX(dx * Math.cos(heading) + dy * Math.sin(heading));
      bodyFramePointToSet.setY(-dx * Math.sin(heading) + dy * Math.cos(heading));
   }

   private void bodyFrameToWorldFrame(Vector2DReadOnly bodyFrameVector, Vector2DBasics worldFrameVectorToSet)
   {
      worldFrameVectorToSet.setX(bodyFrameVector.getX() * Math.cos(heading) - bodyFrameVector.getY() * Math.sin(heading));
      worldFrameVectorToSet.setY(bodyFrameVector.getX() * Math.sin(heading) + bodyFrameVector.getY() * Math.cos(heading));
   }

   private void worldFrameToBodyFrame(Vector2DReadOnly worldFrameVector, Vector2DBasics bodyFrameVectorToSet)
   {
      bodyFrameVectorToSet.setX(worldFrameVector.getX() * Math.cos(heading) + worldFrameVector.getY() * Math.sin(heading));
      bodyFrameVectorToSet.setY(-worldFrameVector.getX() * Math.sin(heading) + worldFrameVector.getY() * Math.cos(heading));
   }

   @Override
   public boolean getDropFlag()
   {
      return inDeliverFlagMode && (xyPosition.getX() > 8.15 && xyPosition.getY() > 8.15);
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      System.out.println("droppedFlag " + flagId);
   }

   @Override
   public void senseHitWall()
   {

   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      if (id == flagIdToChase)
      {
         inDeliverFlagMode = true;
      }
      else if (inDeliverFlagMode)
      {
         inDeliverFlagMode = false;
      }

      flagLocations[id - 1].setToNaN();

      System.out.println("pickedUpFlag " + id);
   }

   private int numberOfRoundsCompleted = 0;

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      System.out.println("deliveredFlag " + flagId);

      if (inDeliverFlagMode)
      {
         if (flagIdToChase == 5)
         {
            flagIdToChase = 1;

            numberOfRoundsCompleted++;
            System.out.println("====================================================");
            System.out.println("             FINISHED " + numberOfRoundsCompleted + " TIMES");
            System.out.println("====================================================");
         }
         else
         {
            flagIdToChase++;
         }

         inDeliverFlagMode = false;
      }
      else
      {
         throw new RuntimeException("Shouldn't get here...");
      }
   }

   private static int clamp(int x, int min, int max)
   {
      return Math.min(Math.max(x, min), max);
   }

   private static double filter(double alpha, double value, double previous)
   {
      return alpha * value + (1.0 - alpha) * previous;
   }

   /////////////////////////////////// entering the xtreme section ///////////////////////////////////////////////////

   static void testBodyWorldTransforms()
   {
      StephenRobot05Behavior behavior = new StephenRobot05Behavior();
      Random random = new Random(38932);

      for (int i = 0; i < 100; i++)
      {
         behavior.heading = random.nextDouble() * Math.PI;
         behavior.xyPosition.setX(random.nextDouble() * 5.0);
         behavior.xyPosition.setY(random.nextDouble() * 5.0);

         // test point conversion

         Point2D bodyFramePointInput = new Point2D(random.nextDouble() * 3.0, random.nextDouble() * 3.0);
         Point2D worldFramePointResult = new Point2D();
         Point2D bodyFramePointOutput = new Point2D();

         behavior.bodyFrameToWorldFrame(bodyFramePointInput, worldFramePointResult);
         behavior.worldFrameToBodyFrame(worldFramePointResult, bodyFramePointOutput);

         Assertions.assertTrue(bodyFramePointInput.epsilonEquals(bodyFramePointOutput, 1e-12));

         // test vector conversion

         Vector2D bodyFrameVectortInput = new Vector2D(random.nextDouble() * 3.0, random.nextDouble() * 3.0);
         Vector2D worldFrameVectorResult = new Vector2D();
         Vector2D bodyFrameVectorOutput = new Vector2D();

         behavior.bodyFrameToWorldFrame(bodyFrameVectortInput, worldFrameVectorResult);
         behavior.worldFrameToBodyFrame(worldFrameVectorResult, bodyFrameVectorOutput);

         Assertions.assertTrue(bodyFrameVectortInput.epsilonEquals(bodyFrameVectorOutput, 1e-12));

      }
   }

   /////////////////////////////////// exiting the xtreme section ///////////////////////////////////////////////////

   public static void main(String[] args)
   {
      testBodyWorldTransforms();
   }

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      // TODO Auto-generated method stub
      
   }

}
