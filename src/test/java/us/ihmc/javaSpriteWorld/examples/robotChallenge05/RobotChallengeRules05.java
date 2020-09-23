package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.Flag;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FlagList;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FoodList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.PredatorList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;

public class RobotChallengeRules05 implements RobotChallengeRules
{
   private final Robot02 robot;
   private final FoodList01 foodList;
   private final PredatorList01 predatorList;
   private final FlagList flagList;
   private final Robot05Behavior robotBehavior;
   private final RobotChallenge01 challenge;

   public RobotChallengeRules05(RobotChallenge01 challenge, Robot02 robot, FoodList01 foodList, PredatorList01 predatorList, FlagList flagList, Robot05Behavior robotBehavior)
   {
      this.challenge = challenge;

      this.robot = robot;
      this.foodList = foodList;
      this.predatorList = predatorList;
      this.flagList = flagList;
      this.robotBehavior = robotBehavior;
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
         robotBehavior.senseGlobalLocation(robot.getX(), robot.getY());
         robotBehavior.senseHeading(robot.getHeading());
         robotBehavior.senseVelocity(robot.getVelocity());

         ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood = foodList.getLocationAndVelocityOfAllFood();
         robotBehavior.senseFood(locationOfAllFood);

         ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators = predatorList.getLocationAndVelocityOfAllPredators();
         robotBehavior.sensePredators(locationOfAllPredators);

         Point2D point = new Point2D(robot.getX(), robot.getY());
         Pair<Point2D, Integer> locationAndIdOfClosestFlag = flagList.getLocationAndIdOfClosestFlag(point);
         robotBehavior.senseClosestFlag(locationAndIdOfClosestFlag);

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
   
   @Override
   public void droppedFlag(int id)
   {
      robotBehavior.droppedFlag(id);
   }

   @Override
   public void capturedFlag(int id)
   {
      robotBehavior.pickedUpFlag(id);
   }
   
   @Override
   public void deliveredFlag(int flagId)
   {
      robotBehavior.deliveredFlag(flagId); 
   }

}
