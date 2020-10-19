package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

public interface RobotChallengeRules
{
   public abstract void executeRules();

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void droppedFlag(int id);

   public abstract void capturedFlag(int id);

   public abstract void deliveredFlag(int id);

   public abstract void hitWall();

   public abstract void senseKeyPressed(String keyPressed);

   public abstract void reportScoreHealthTime(double score, double health, double time);

   public abstract void reset();

}
