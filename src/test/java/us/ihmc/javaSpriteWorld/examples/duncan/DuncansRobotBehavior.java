package us.ihmc.javaSpriteWorld.examples.duncan;

public class DuncansRobotBehavior extends FunctionalRobotBehaviorAdapter
{
   private final Environment environment;

   public DuncansRobotBehavior(int challengeNumber)
   {
      environment = new Environment(challengeNumber);
   }
}
