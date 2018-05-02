package us.ihmc.javaSpriteWorld;

public class SpriteStage extends Sprite
{
   public SpriteStage(String name)
   {
      super(name);
   }
   
   public void setCenteredInSpriteWorld(SpriteWorld spriteWorld)
   {
      double leftBorderX = spriteWorld.getLeftBorderX();
      double rightBorderX = spriteWorld.getRightBorderX();
      double topBorderY = spriteWorld.getTopBorderY();
      double bottomBorderY = spriteWorld.getBottomBorderY();
      
      this.setRotationInDegrees(0.0);
      this.setX(0.5 * (leftBorderX + rightBorderX));
      this.setY(0.5 * (topBorderY + bottomBorderY));
      
      this.setWidth(Math.abs(rightBorderX - leftBorderX));
      this.setHeight(Math.abs(topBorderY - bottomBorderY));
   }

   public int addBackdrop(String filename)
   {
      SpriteCostume costume = SpriteCostume.createFromFile(filename);
      StageBackdrop stageBackdrop = new StageBackdrop(costume);
      
      return this.addBackdrop(stageBackdrop);
   }

   public int addBackdrop(StageBackdrop stageBackdrop)
   {
      return this.addCostume(stageBackdrop);
   }

   public void switchToBackdrop(int backdropNumber)
   {
      this.switchToCostume(backdropNumber);
   }

   public StageBackdrop getBackdrop(int backdropNumber)
   {
      return (StageBackdrop) this.getCostume(backdropNumber);
   }
   
   public StageBackdrop getBackdrop()
   {
      return (StageBackdrop) this.getCostume();
   }
}
