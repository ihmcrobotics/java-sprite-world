package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.Flag;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FlagList;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FoodList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.PredatorList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeTools;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;

/*
 * Sense in Body Frame instead of World Frame.
 */
public class RobotChallengeRules05 implements RobotChallengeRules
{
   private final Robot02 robot;
   private final FoodList01 foodList;
   private final PredatorList01 predatorList;
   private final FlagList flagList;
   private final Robot05Behavior robotBehavior;
   private final RobotChallenge01 challenge;

   private boolean testing = false;
   
   public RobotChallengeRules05(RobotChallenge01 challenge, Robot02 robot, FoodList01 foodList, PredatorList01 predatorList, FlagList flagList,
                                Robot05Behavior robotBehavior)
   {
      this.challenge = challenge;

      this.robot = robot;
      this.foodList = foodList;
      this.predatorList = predatorList;
      this.flagList = flagList;
      this.robotBehavior = robotBehavior;
   }
   
   public void setTesting(boolean testing)
   {
      this.testing = testing;
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      robotBehavior.senseMousePressed(mousePressedX, mousePressedY);
   }

   @Override
   public void executeRules()
   {
      if (robotBehavior != null)
      {
         if (testing)
            robotBehavior.senseGlobalPositionForTestingOnly(robot.getX(), robot.getY());
         
         ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame = senseWallRangeFinderPointsInBodyFrame();
         robotBehavior.senseWallRangeInBodyFrame(vectorsAndDistancesToWallInBodyFrame);

         robotBehavior.senseHeading(senseRobotHeading());
         robotBehavior.senseVelocity(senseRobotVelocity());

         ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFoodInBodyFrame = senseLocationOfFoodInBodyFrame();
         robotBehavior.senseFoodInBodyFrame(locationOfAllFoodInBodyFrame);

         ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredatorsInBodyFrame = senseLocationOfPredatorsInBodyFrame();
         robotBehavior.sensePredatorsInBodyFrame(locationOfAllPredatorsInBodyFrame);

         if (!flagList.getLocationAndIdsOfAllFlags().isEmpty())
         {
            Pair<Point2D, Integer> vectorToInBodyFrameAndIdOfClosestFlag = senseVectorToClosestFlagInBodyFrame();
            robotBehavior.senseClosestFlagInBodyFrame(vectorToInBodyFrameAndIdOfClosestFlag);
         }

         double[] accelerationAndTurnRate = robotBehavior.getAccelerationAndTurnRate();

         robot.setAcceleration(accelerationAndTurnRate[0]);
         robot.setTurnRate(accelerationAndTurnRate[1]);

         boolean dropFlag = robotBehavior.getDropFlag();

         if (dropFlag)
         {
            Flag flagDropped = robot.dropFlag();

            if (flagDropped != null)
            {
               challenge.robotDroppedFlag(flagDropped);
            }

         }
      }
   }

   protected ArrayList<Pair<Vector2D, Double>> senseWallRangeFinderPointsInBodyFrame()
   {
      double[] rangeSensorAngles = new double[] {-4.0/8.0*Math.PI, -3.0/8.0*Math.PI, -2.0/8.0*Math.PI, -1.0/8.0*Math.PI, 0.0, 1.0/8.0*Math.PI, 2.0/8.0*Math.PI, 3.0/8.0*Math.PI, 4.0/8.0*Math.PI}; 
      ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame = new ArrayList<Pair<Vector2D, Double>>();

      for (double rangeSensorAngle : rangeSensorAngles)
      {
         Pair<Vector2D, Double> vectorAndDistance = senseWallDistanceGivenSensingVectorInBody(rangeSensorAngle);
         vectorsAndDistancesToWallInBodyFrame.add(vectorAndDistance);
      }

      return vectorsAndDistancesToWallInBodyFrame;
   }

   protected ArrayList<Pair<Point2D, Vector2D>> senseLocationOfPredatorsInBodyFrame()
   {
      ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators = predatorList.getLocationAndVelocityOfAllPredators();
      ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredatorsInBodyFrame = RobotChallengeTools.convertFromWorldToBodyFrame(robot.getPosition(),
                                                                                                                             locationOfAllPredators,
                                                                                                                             senseRobotHeading());
      return locationOfAllPredatorsInBodyFrame;
   }

   protected ArrayList<Triple<Integer, Point2D, Vector2D>> senseLocationOfFoodInBodyFrame()
   {
      ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood = foodList.getLocationAndVelocityOfAllFood();
      ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFoodInBodyFrame = RobotChallengeTools.convertFromWorldToBodyFrameKeepingIds(robot.getPosition(),
                                                                                                                        locationOfAllFood,
                                                                                                                        senseRobotHeading());
      return locationOfAllFoodInBodyFrame;
   }

   protected double senseRobotVelocity()
   {
      return robot.getVelocity();
   }

   protected double senseRobotHeading()
   {
      return robot.getHeading();
   }

   private Pair<Vector2D, Double> senseWallDistanceGivenSensingVectorInBody(double rangeAngleInBody)
   {
      double totalAngle = robot.getHeading() + rangeAngleInBody;
      Vector2D rangeVectorInBody = new Vector2D(0.0, 1.0);
      Vector2D rangeVectorInWorld = new Vector2D(0.0, 1.0);
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(rangeAngleInBody, 0.0, 0.0);
      transform.transform(rangeVectorInBody);
      
      transform.getRotation().setYawPitchRoll(totalAngle, 0.0, 0.0);
      transform.transform(rangeVectorInWorld);

      Point2DBasics intersectionWithWall = challenge.getIntersectionWithWall(robot.getPosition(), rangeVectorInWorld);
      double wallDistance = Double.POSITIVE_INFINITY;
      
      if (intersectionWithWall != null)
         wallDistance = intersectionWithWall.distance(robot.getPosition());

      ImmutablePair<Vector2D, Double> vectorAndDistance = new ImmutablePair<Vector2D, Double>(rangeVectorInBody, wallDistance);

      return vectorAndDistance;
   }

   protected Pair<Point2D, Integer> senseVectorToClosestFlagInBodyFrame()
   {
      Point2D robotLocationInWorld = robot.getPosition();
      Pair<Point2D, Integer> locationAndIdOfClosestFlag = flagList.getLocationAndIdOfClosestFlag(robotLocationInWorld);
      Point2D worldLocationOfFlag = locationAndIdOfClosestFlag.getLeft();

      Point2D vectorToFlagInBody = RobotChallengeTools.computePositionInRobotBodyFrame(robotLocationInWorld, worldLocationOfFlag, senseRobotHeading());

      Pair<Point2D, Integer> positionOfClosestFlagInBodyFrame = new ImmutablePair<Point2D, Integer>(vectorToFlagInBody, locationAndIdOfClosestFlag.getRight());
      return positionOfClosestFlagInBodyFrame;
   }

   @Override
   public void droppedFlag(int id)
   {
      robotBehavior.senseDroppedFlag(id);
   }

   @Override
   public void capturedFlag(int id)
   {
      robotBehavior.sensePickedUpFlag(id);
   }

   @Override
   public void deliveredFlag(int flagId)
   {
      robotBehavior.senseDeliveredFlag(flagId);
   }

   @Override
   public void hitWall()
   {
      robotBehavior.senseHitWall();
   }
}
