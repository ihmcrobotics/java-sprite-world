package us.ihmc.javaSpriteWorld.examples.robotChallenge06;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FlagList;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FoodList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.PredatorList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeTools;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.RobotChallengeRules05;

public class RobotChallengeRules06 extends RobotChallengeRules05
{

   public RobotChallengeRules06(RobotChallenge01 challenge, Robot02 robot, FoodList01 foodList, PredatorList01 predatorList, FlagList flagList,
                                Robot05Behavior robotBehavior)
   {
      super(challenge, robot, foodList, predatorList, flagList, robotBehavior);
   }
   
   @Override
   protected ArrayList<Pair<Point2D, Vector2D>> senseLocationOfPredatorsInBodyFrame()
   {
      ArrayList<Pair<Point2D, Vector2D>> locationOfPredatorsInBodyFrame = super.senseLocationOfPredatorsInBodyFrame();
   
   
      return locationOfPredatorsInBodyFrame;
   }

   protected ArrayList<Pair<Point2D, Vector2D>> senseLocationOfFood()
   {
      ArrayList<Pair<Point2D, Vector2D>> locationOfFood = super.senseLocationOfFood();
      
      return locationOfFood;
   }

   protected double senseRobotVelocity()
   {
      double robotVelocity = super.senseRobotVelocity();
      
      return robotVelocity;
   }

   protected double senseRobotHeading()
   {
      double robotHeading = super.senseRobotHeading();
      return robotHeading;
   }

   protected double senseWallDistanceStraightAhead()
   {
     double wallDistanceStraightAhead = super.senseWallDistanceStraightAhead();
     return wallDistanceStraightAhead;
   }

   protected Pair<Point2D, Integer> senseVectorToClosestFlagInBodyFrame()
   {
      Pair<Point2D, Integer> vectorToClosestFlagInBodyFrame = super.senseVectorToClosestFlagInBodyFrame();
   
      return vectorToClosestFlagInBodyFrame;
   }

  
}
