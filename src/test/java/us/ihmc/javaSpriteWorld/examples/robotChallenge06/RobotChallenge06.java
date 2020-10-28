package us.ihmc.javaSpriteWorld.examples.robotChallenge06;

import java.util.HashMap;
import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.SimpleRobot05Behavior;
import us.ihmc.javaSpriteWorld.examples.stephen.StephenRobotBehavior;

/**
 * Adds noise to the sensors.
 */
public class RobotChallenge06
{

   public static void main(String[] args)
   {
//      String player = System.getProperty("player", "Stephen");
//      String player = System.getProperty("player", "Simple");
      String[] players = {"Stephen", "Duncan"};
      
      HashMap<String, RobotChallenge01> robotChallenges = new HashMap<String, RobotChallenge01>();
      HashMap<String, double[]> scores = new HashMap<String, double[]>();
      
      for (String player : players)
      {
         RobotChallenge01 robotChallenge = createRobotChallenge(player);
         robotChallenges.put(player, robotChallenge);
      }

      int numberOfRuns = 5;
      double realTimeSpeedup = 20.0;
      double simulationTime = 60.0;

      for (String player : players)
      {
         RobotChallenge01 robotChallenge = robotChallenges.get(player);

         double[] averageAndTopScore = robotChallenge.runATrial(numberOfRuns, realTimeSpeedup, simulationTime);
         scores.put(player, averageAndTopScore);
      }

      for (String player : players)
      {
         RobotChallenge01 robotChallenge = robotChallenges.get(player);
         robotChallenge.exit();

         double[] score = scores.get(player);
         System.out.println(player + " scores: " + score[0] + ", " + score[1]);
      }
   }

   private static RobotChallenge01 createRobotChallenge(String player)
   {
      double xMax = 10.0;
      double yMax = 10.0;
      Random random = new Random();
      
      Robot06Behavior robotBehavior = createBehavior(player, xMax, yMax);
      Robot02 robot = new Robot02(xMax, yMax);
      RobotChallenge01 robotChallenge = createRobotChallenge(player, random, xMax, yMax, robot);

      RobotChallenge06NoiseParameters noiseParameters = new RobotChallenge06NoiseParameters();
//      noiseParameters.removeAllNoise();
      RobotChallengeRules06 rules = new RobotChallengeRules06(random, noiseParameters, robotChallenge, robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), robotChallenge.getFlagList(), robotBehavior);
      
      if (player == "Simple")
         rules.setTesting(true);

      robotChallenge.setRobotChallengeRules(rules);
      return robotChallenge;
   }

   private static RobotChallenge01 createRobotChallenge(String player, Random random, double xMax, double yMax, Robot02 robot)
   {
      RobotChallenge01 robotChallenge = new RobotChallenge01("RobotChallenge06: " + player, robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(3, maximumPredatorSpeed);
      robotChallenge.createSomeFlags(5);
      return robotChallenge;
   }

   private static Robot06Behavior createBehavior(String player, double xMax, double yMax)
   {
      Robot06Behavior simpleBehavior;
      if (player.equals("Duncan"))
         simpleBehavior = new DuncanRobot06Behavior();
      else if (player.equals("Stephen"))
         simpleBehavior = new StephenRobotBehavior();
      else
         simpleBehavior = new SimpleRobot05Behavior(xMax, yMax);
      return simpleBehavior;
   }

}
