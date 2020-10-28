package us.ihmc.javaSpriteWorld.examples.robotChallenge08;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.SimpleRobot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.RobotChallenge06NoiseParameters;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.RobotChallengeRules06;

import us.ihmc.javaSpriteWorld.examples.stephen.StephenRobotBehavior;
/**
 * Add some rooms and make the environment larger.
 *
 */
public class RobotChallenge08
{
   public static final boolean DEBUGGING_MODE = Boolean.parseBoolean(System.getProperty("debug", "false"));

   public static void main(String[] args)
   {
    String player = System.getProperty("player", "Simple");
    RobotChallenge01 robotChallenge = createRobotChallenge(player);
    if (DEBUGGING_MODE)
    {
       robotChallenge.runSimulation();
    }

    int numberOfRuns = 30;
    double realTimeSpeedup = 20.0;
    double simulationTime = 60.0;
    robotChallenge.runATrial(numberOfRuns, realTimeSpeedup, simulationTime);
   }

   private static RobotChallenge01 createRobotChallenge(String player)
   {
      double xMax = 20.0;
      double yMax = 20.0;
      Random random = new Random();
      
      Robot06Behavior robotBehavior = createBehavior(player, xMax, yMax);
      Robot02 robot = new Robot02(xMax, yMax);
      RobotChallenge01 robotChallenge = createRobotChallenge(xMax, yMax, robot, random);

      RobotChallenge06NoiseParameters noiseParameters = new RobotChallenge06NoiseParameters();
      RobotChallengeRules06 rules = new RobotChallengeRules06(random, noiseParameters, robotChallenge, robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), robotChallenge.getFlagList(), robotBehavior);

      if (player.equals("Simple"))
         rules.setTesting(true);

      robotChallenge.setRobotChallengeRules(rules);
      return robotChallenge;
   }

   private static RobotChallenge01 createRobotChallenge(double xMax, double yMax, Robot02 robot, Random random)
   {
      RobotChallenge01 robotChallenge = new RobotChallenge01("RobotChallenge08", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(30);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(3, maximumPredatorSpeed);
      robotChallenge.createSomeFlags(6);
      robotChallenge.createSomeRooms();
      return robotChallenge;
   }

   private static Robot06Behavior createBehavior(String player, double xMax, double yMax)
   {
      Robot06Behavior robotBehavior;
      if (player.equals("Duncan"))
         robotBehavior = new DuncanRobot08Behavior(DEBUGGING_MODE);
      else if (player.equals("Stephen"))
         robotBehavior = new StephenRobotBehavior();
      else
         robotBehavior = new SimpleRobot05Behavior(xMax, yMax);
      return robotBehavior;
   }

}
