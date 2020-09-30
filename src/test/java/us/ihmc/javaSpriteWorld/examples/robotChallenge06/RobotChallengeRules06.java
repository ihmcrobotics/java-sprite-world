package us.ihmc.javaSpriteWorld.examples.robotChallenge06;

import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FlagList;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FoodList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.PredatorList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.RobotChallengeRules05;

/**
 * Adds noise to the sensors.
 */
public class RobotChallengeRules06 extends RobotChallengeRules05
{
   private final Random random;
   private final RobotChallenge06NoiseParameters noiseParameters;
   private double robotVelocityBias = 0.0;

   public RobotChallengeRules06(Random random, RobotChallenge06NoiseParameters noiseParameters, RobotChallenge01 challenge, Robot02 robot, FoodList01 foodList,
                                PredatorList01 predatorList, FlagList flagList, Robot05Behavior robotBehavior)
   {
      super(challenge, robot, foodList, predatorList, flagList, robotBehavior);
      this.random = random;
      this.noiseParameters = noiseParameters;
   }

   @Override
   protected ArrayList<Pair<Point2D, Vector2D>> senseLocationOfPredatorsInBodyFrame()
   {
      ArrayList<Pair<Point2D, Vector2D>> locationOfPredatorsInBodyFrame = super.senseLocationOfPredatorsInBodyFrame();
      ArrayList<Pair<Point2D, Vector2D>> noisyLocationOfPredatorsInBodyFrame = addNoise(locationOfPredatorsInBodyFrame,
                                                                                        noiseParameters.getPredatorPositionNoiseMagnitude(),
                                                                                        noiseParameters.getPredatorVelocityNoiseMagnitude());

      return noisyLocationOfPredatorsInBodyFrame;
   }

   protected ArrayList<Pair<Point2D, Vector2D>> senseLocationOfFoodInBodyFrame()
   {
      ArrayList<Pair<Point2D, Vector2D>> locationOfFood = super.senseLocationOfFoodInBodyFrame();

      return locationOfFood;
   }

   protected double senseRobotVelocity()
   {
      double robotVelocity = super.senseRobotVelocity();

      robotVelocityBias = addNoise(robotVelocityBias, RobotChallenge01.dt * 1.0);
      robotVelocityBias = limitMagnitude(robotVelocityBias, noiseParameters.getRobotVelocityBiasMagnitude());

      double noisyRobotVelocity = robotVelocityBias + addNoise(robotVelocity, noiseParameters.getRobotVelocityNoiseMagnitude());
      return noisyRobotVelocity;
   }

   private double limitMagnitude(double data, double magnitudeLimit)
   {
      if (data < -magnitudeLimit)
         return -magnitudeLimit;
      if (data > magnitudeLimit)
         return magnitudeLimit;
      return data;
   }

   protected double senseRobotHeading()
   {
      double robotHeading = super.senseRobotHeading();
      double noisyRobotHeading = addNoise(robotHeading, noiseParameters.getRobotHeadingNoiseMagnitude());

      return noisyRobotHeading;
   }

   protected double senseWallDistanceStraightAhead()
   {
      double wallDistanceStraightAhead = super.senseWallDistanceStraightAhead();
      double noisyWallDistance = addNoise(wallDistanceStraightAhead, noiseParameters.getWallDistanceNoiseMagnitude());

      return noisyWallDistance;
   }

   protected Pair<Point2D, Integer> senseVectorToClosestFlagInBodyFrame()
   {
      Pair<Point2D, Integer> vectorToClosestFlagInBodyFrame = super.senseVectorToClosestFlagInBodyFrame();
      Pair<Point2D, Integer> noisyVectorToClosestFlagInBodyFrame = addNoise(vectorToClosestFlagInBodyFrame, noiseParameters.getFlagPositionNoiseMagnitude());

      return noisyVectorToClosestFlagInBodyFrame;
   }

   private ArrayList<Pair<Point2D, Vector2D>> addNoise(ArrayList<Pair<Point2D, Vector2D>> pointAndVectorList, double pointNoiseMagnitude,
                                                       double vectorNoiseMagnitude)
   {
      ArrayList<Pair<Point2D, Vector2D>> noisyData = new ArrayList<Pair<Point2D, Vector2D>>();

      for (Pair<Point2D, Vector2D> pointAndVector : pointAndVectorList)
      {
         Point2D noisyPoint = addNoise(pointAndVector.getLeft(), pointNoiseMagnitude);
         Vector2D noisyVector = addNoise(pointAndVector.getRight(), vectorNoiseMagnitude);
         Pair<Point2D, Vector2D> noisyPointAndVector = new ImmutablePair<Point2D, Vector2D>(noisyPoint, noisyVector);
         noisyData.add(noisyPointAndVector);
      }

      return noisyData;
   }

   private Pair<Point2D, Integer> addNoise(Pair<Point2D, Integer> data, double pointNoiseMagnitude)
   {
      Point2D noisyPoint = addNoise(data.getLeft(), pointNoiseMagnitude);
      Pair<Point2D, Integer> noisyData = new ImmutablePair<Point2D, Integer>(noisyPoint, data.getRight());

      return noisyData;
   }

   private Vector2D addNoise(Vector2D vector, double vectorNoiseMagnitude)
   {
      double noisyX = addNoise(vector.getX(), vectorNoiseMagnitude);
      double noisyY = addNoise(vector.getX(), vectorNoiseMagnitude);
      return new Vector2D(noisyX, noisyY);
   }

   private Point2D addNoise(Point2D point, double pointNoiseMagnitude)
   {
      double noisyX = addNoise(point.getX(), pointNoiseMagnitude);
      double noisyY = addNoise(point.getX(), pointNoiseMagnitude);
      return new Point2D(noisyX, noisyY);
   }

   private double addNoise(double data, double noiseMagnitude)
   {
      double noisyData = data + (1.0 - 2.0 * random.nextDouble()) * noiseMagnitude;
      return noisyData;
   }

}
