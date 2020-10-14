package us.ihmc.javaSpriteWorld.examples.robotChallenge07;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.DuncanRobot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.SimpleRobot05Behavior;
import us.ihmc.javaSpriteWorld.examples.stephen.StephenRobotBehavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.RobotChallenge06NoiseParameters;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.RobotChallengeRules06;

/**
 * Add a wall.
 *
 */
public class RobotChallenge07
{
   public static final String PLAYER = System.getProperty("player", "Simple");

   public static void main(String[] args)
   {
      Random random = new Random();
      double xMax = 10.0;
      double yMax = 10.0;
      
      Robot02 robot = new Robot02(xMax, yMax);

      RobotChallenge01 robotChallenge = new RobotChallenge01("RobotChallenge07", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(3, maximumPredatorSpeed);
      robotChallenge.createSomeFlags(5);
      robotChallenge.createAWall();

      Robot06Behavior simpleBehavior;
      if (PLAYER.equals("Duncan"))
         simpleBehavior = new DuncanRobot05Behavior();
      else if (PLAYER.equals("Stephen"))
         simpleBehavior = new StephenRobotBehavior();
      else
         simpleBehavior = new SimpleRobot05Behavior(xMax, yMax);
      
      RobotChallenge06NoiseParameters noiseParameters = new RobotChallenge06NoiseParameters();
      RobotChallengeRules06 rules = new RobotChallengeRules06(random, noiseParameters, robotChallenge, robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), robotChallenge.getFlagList(), simpleBehavior);
//      rules.setTesting(true);

      robotChallenge.setRootChallengeRules(rules);
      robotChallenge.runSimulation();
   }

}
