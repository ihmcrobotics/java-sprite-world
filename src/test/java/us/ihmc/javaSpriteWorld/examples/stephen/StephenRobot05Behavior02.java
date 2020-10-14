package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import org.junit.jupiter.api.Assertions;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class StephenRobot05Behavior02 implements Robot05Behavior, Robot06Behavior
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
   private double wallDistance;

   // behavior parameters
   private final List<ObjectResponseDescription> responseDescriptions = new ArrayList<>();
   private final double[] totalAction = new double[2];

   private final double wallAngularCostRange = Math.toRadians(75.0);
   private final double foodAngularCostRange = Math.toRadians(180.0);
   private final double predatorAngularCostRange = Math.toRadians(80.0);
   private final double goToFlagAngularCostRange = Math.toRadians(180.0);
   private final double avoidFlagAngularCostRange = Math.toRadians(45.0);

   private final double maxForceWall = 2.5;
   private final double maxForceFood = 1.0;
   private final double maxForcePredator = 1.2;
   private final double maxForceAvoidFlagWhileDelivering = 3.0;
   private final double flagAttractionForceMagnitude = 0.5;
   private final double exploreAreaForceMagnitude = 0.4;

   private final double minAngleToPenalizeFood = Math.toRadians(60.0);
   private final double extraFoodDistanceIfBehind = 1.0;
   private final double proximityNearWallToIgnoreFood = 0.8;

   private final Vector2D[] flagLocations = new Vector2D[5];

   private final Point2D[] areasToExplore;
   private int counter = 0;
   private final int counterToSwitchExploreArea = 500;
   private final double proximityToExploreAreaToSwitch = 0.4;
   private final Point2D areaToExplore = new Point2D();
   private final Random random = new Random(3920);

   private final double baseWall = 2.65;
   private double baseFood = 2.2;
   private final double basePredator = 2.0;
   private final double baseFlag = 2.5;

   private int flagIdToChase = 1;
   private boolean inDeliverFlagMode = false;

   public StephenRobot05Behavior02()
   {
      for (int i = 0; i < flagLocations.length; i++)
      {
         flagLocations[i] = new Vector2D();
         flagLocations[i].setToNaN();
      }

      areasToExplore = new Point2D[9];
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            areasToExplore[3*i + j] = new Point2D(5 + 3 * (i - 1), 5 + 3 * (j - 1));
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
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {

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

      responseDescriptions.clear();

      // wall
      computeWallAction();

      // food
      for (int i = 0; i < locationOfAllFoodInBodyFrame.size(); i++)
      {
         computeFoodAction(locationOfAllFoodInBodyFrame.get(i).getMiddle());
      }

      // predators
      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         computePredatorForce(locationOfAllPredators.get(i).getLeft());
      }

      // chase flag
      if (inDeliverFlagMode)
      {
         Vector2D dropPointWorld = new Vector2D(9.0, 9.0);
         Vector2D dropPointBody = new Vector2D();
         worldFrameToBodyFrame(dropPointWorld, dropPointBody);

         computeGoToFlagForce(dropPointBody);
         computeAvoidFlagForce(positionInBodyFrameAndIdOfClosestFlag.getLeft());
      }
      else
      {
         boolean detectedFlagShouldBeRetrieved = positionInBodyFrameAndIdOfClosestFlag.getRight() == flagIdToChase;
         if (detectedFlagShouldBeRetrieved)
         {
            computeGoToFlagForce(positionInBodyFrameAndIdOfClosestFlag.getLeft());
         }
         else
         {
            if (flagLocations[flagIdToChase - 1].containsNaN())
            {
               computeGoToWorldCoordinate(areaToExplore, exploreAreaForceMagnitude, goToFlagAngularCostRange);
            }
            else
            {
               computeGoToWorldCoordinate(flagLocations[flagIdToChase - 1], flagAttractionForceMagnitude, goToFlagAngularCostRange);
            }
         }
      }

      // find max cost (rename later...) heading
      double angle = -Math.PI;
      double maxCost = Double.NEGATIVE_INFINITY;
      double maxCostHeading = Double.NaN;

      while (angle <= Math.PI)
      {
         double cost = 0.0;
         for (int i = 0; i < responseDescriptions.size(); i++)
         {
            cost += responseDescriptions.get(i).getCostAtAngle(angle);
         }

         if (cost > maxCost)
         {
            maxCost = cost;
            maxCostHeading = angle;
         }

         angle += 0.01;
      }

      double velocityWhenAligned = 3.0;
      double targetVelocity;
      double angleToStopAndTurn = Math.toRadians(60.0);
      double deltaDesiredHeading = maxCostHeading;

      if (Math.abs(deltaDesiredHeading) < angleToStopAndTurn)
      {
         targetVelocity = velocityWhenAligned - (velocityWhenAligned / angleToStopAndTurn) * Math.abs(deltaDesiredHeading);
      }
      else
      {
         targetVelocity = 0.0;
      }

      double kAcceleration = 3.0;
      double accelerationAction = kAcceleration * (targetVelocity - velocity);

      double kTurn = 4.0;
      double turningAction = kTurn * deltaDesiredHeading;

      totalAction[0] = accelerationAction;
      totalAction[1] = turningAction;

      return totalAction;
   }

   private void doLocalization()
   {
      Point2D wallPointInBody = new Point2D(0.0, wallDistance);
      Point2D wallPointInWorldEstimatedFrame = new Point2D();
      bodyFrameToWorldFrame(wallPointInBody, wallPointInWorldEstimatedFrame);

      for (int i = 0; i < wallPoints.size(); i++)
      {
         Point2D expectedIntersectionPoint = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(xyPosition,
                                                                                             xyHeading,
                                                                                             wallPoints.get(i),
                                                                                             wallPoints.get((i + 1) % wallPoints.size()));
         if (expectedIntersectionPoint != null)
         {
            Vector2D correctionVector = new Vector2D(expectedIntersectionPoint);
            correctionVector.sub(wallPointInWorldEstimatedFrame);
            xyPosition.add(correctionVector);
            return;
         }
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

   private void computeWallAction()
   {
      double cost = -maxForceWall * Math.pow(baseWall, -wallDistance);
      responseDescriptions.add(new ObjectResponseDescription(0.0, wallAngularCostRange, cost));
   }

   private void computeFoodAction(Tuple2DReadOnly foodInBodyFrame)
   {
      double distanceToWall = computeDistanceToWall(foodInBodyFrame);
      if (distanceToWall < proximityNearWallToIgnoreFood)
      {
         return;
      }

      double distance = EuclidCoreTools.norm(foodInBodyFrame.getX(), foodInBodyFrame.getY());
      double angleFromStraightAhead = headingFromVector(foodInBodyFrame);
      if (Math.abs(angleFromStraightAhead) > minAngleToPenalizeFood)
      {
         double angleFromStraightBehind = EuclidCoreTools.angleDifferenceMinusPiToPi(angleFromStraightAhead, Math.PI);
         distance += extraFoodDistanceIfBehind * (1.0 - Math.abs(angleFromStraightBehind) / (Math.PI - minAngleToPenalizeFood));
      }

      double cost = maxForceFood * Math.pow(baseFood, - distance);
      double heading = headingFromVector(foodInBodyFrame);
      responseDescriptions.add(new ObjectResponseDescription(heading, foodAngularCostRange, cost));
   }

   private void computePredatorForce(Tuple2DReadOnly predatorInBodyFrame)
   {
      double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
      double cost = - maxForcePredator * Math.pow(basePredator, - distance);
      double heading = headingFromVector(predatorInBodyFrame);
      responseDescriptions.add(new ObjectResponseDescription(heading, predatorAngularCostRange, cost));
   }

   private void computeAvoidFlagForce(Tuple2DReadOnly flagInBodyFrame)
   {
      double heading = headingFromVector(flagInBodyFrame);
      double distance = EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY());
      double cost = - maxForceAvoidFlagWhileDelivering * Math.pow(baseFlag, - distance);
      responseDescriptions.add(new ObjectResponseDescription(heading, avoidFlagAngularCostRange, cost));
   }

   private void computeGoToFlagForce(Tuple2DReadOnly flagInBodyFrame)
   {
      responseDescriptions.add(new ObjectResponseDescription(headingFromVector(flagInBodyFrame), goToFlagAngularCostRange, flagAttractionForceMagnitude));
   }

   private void computeGoToWorldCoordinate(Tuple2DReadOnly worldCoordinate, double cost, double angularRange)
   {
      Point2D bodyFrameCoordinate = new Point2D();
      worldFrameToBodyFrame(worldCoordinate, bodyFrameCoordinate);
      double heading = headingFromVector(bodyFrameCoordinate);
      responseDescriptions.add(new ObjectResponseDescription(heading, angularRange, cost));
   }

   private double computeDistanceToWall(Tuple2DReadOnly foodInBodyFrame)
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

   private void bodyFrameToWorldFrame(Tuple2DReadOnly bodyFrameVector, Tuple2DBasics worldFrameVectorToSet)
   {
      double relativeWorldVectorX = bodyFrameVector.getX() * Math.cos(heading) - bodyFrameVector.getY() * Math.sin(heading);
      double relativeWorldVectorY = bodyFrameVector.getX() * Math.sin(heading) + bodyFrameVector.getY() * Math.cos(heading);

      worldFrameVectorToSet.setX(relativeWorldVectorX + xyPosition.getX());
      worldFrameVectorToSet.setY(relativeWorldVectorY + xyPosition.getY());
   }

   private void worldFrameToBodyFrame(Tuple2DReadOnly worldFrameVector, Tuple2DBasics bodyFrameVectorToSet)
   {
      double dx = worldFrameVector.getX() - xyPosition.getX();
      double dy = worldFrameVector.getY() - xyPosition.getY();

      bodyFrameVectorToSet.setX(dx * Math.cos(heading) + dy * Math.sin(heading));
      bodyFrameVectorToSet.setY(-dx * Math.sin(heading) + dy * Math.cos(heading));
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

   @Override
   public void senseHitWall()
   {

   }

   private class ObjectResponseDescription
   {
      private final double heading;
      private final double angularRange;
      private final double cost;

      public ObjectResponseDescription(double heading, double angularRange, double cost)
      {
         this.heading = heading;
         this.angularRange = angularRange;
         this.cost = cost;
      }

      public double getHeading()
      {
         return heading;
      }

      public double getAngularRange()
      {
         return angularRange;
      }

      public double getCost()
      {
         return cost;
      }

      public double getCostAtAngle(double angle)
      {
         double angularDifference = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(angle, heading));
         if (angularDifference > angularRange)
         {
            return 0.0;
         }
         else
         {
            return cost * (1.0 - angularDifference / angularRange);
         }
      }
   }

   private static int clamp(int x, int min, int max)
   {
      return Math.min(Math.max(x, min), max);
   }

   /////////////////////////////////// entering the xtreme section ///////////////////////////////////////////////////

   static void testBodyWorldTransforms()
   {
      StephenRobot05Behavior02 behavior = new StephenRobot05Behavior02();
      Random random = new Random(38932);

      for (int i = 0; i < 100; i++)
      {
         behavior.heading = random.nextDouble() * Math.PI;
         behavior.xyPosition.setX(random.nextDouble() * 5.0);
         behavior.xyPosition.setY(random.nextDouble() * 5.0);

         Vector2D bodyFrameInput = new Vector2D(random.nextDouble() * 3.0, random.nextDouble() * 3.0);
         Vector2D worldFrameResult = new Vector2D();
         Vector2D bodyFrameOutput = new Vector2D();

         behavior.bodyFrameToWorldFrame(bodyFrameInput, worldFrameResult);
         behavior.worldFrameToBodyFrame(worldFrameResult, bodyFrameOutput);

         Assertions.assertTrue(bodyFrameInput.epsilonEquals(bodyFrameOutput, 1e-12));
      }
   }

   /////////////////////////////////// exiting the xtreme section ///////////////////////////////////////////////////

   public static void main(String[] args)
   {
      testBodyWorldTransforms();
   }
}
