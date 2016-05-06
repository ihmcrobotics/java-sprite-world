package us.ihmc.javaSpriteWorld;

import java.awt.Color;

public class SampleSprites 
{

	public static Sprite createSixSidedBlackPipsOnWhiteDie()
	{
		Sprite sprite = new Sprite("SixSidedBlackPipsOnWhiteDie");
		
		SpriteCostume spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/blackOne-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/blackTwo-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/blackThree-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/blackFour-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/blackFive-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/blackSix-300px.png");
		sprite.addCostume(spriteCostume);
		
		return sprite;
	}
	
	public static Sprite createSixSidedRedPipsOnWhiteDie()
	{
		Sprite sprite = new Sprite("SixSidedRedPipsOnWhiteDie");
		
		SpriteCostume spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/dado-1-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/dado-2-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/dado-3-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/dado-4-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/dado-5-300px.png");
		sprite.addCostume(spriteCostume);
		
		spriteCostume = SpriteCostume.createFromFile("SampleImages/dice/SixSider/dado-6-300px.png");
		sprite.addCostume(spriteCostume);
		
		return sprite;
	}
	
	public static Sprite createRocketOne()
	{		
		Sprite sprite = new Sprite("RocketOne");
		
		SpriteCostume spriteCostume = SpriteCostume.createFromFile("SampleImages/Rockets/purzen-A-cartoon-moon-rocket-800px.png");
		sprite.addCostume(spriteCostume);
		
		return sprite;
	}
	
	public static Sprite createRocketFlameOne()
	{		
		Sprite sprite = new Sprite("RocketFlame");
		
		SpriteCostume spriteCostume = SpriteCostume.createFromFile("SampleImages/Rockets/racing-flame-300px.png");
		spriteCostume.setXReferencePercent(0.5);
		spriteCostume.setYReferencePercent(0.0); // top of flames
		sprite.addCostume(spriteCostume);
		
		return sprite;
	}
	
	public static Sprite createRectangle(double width, double height, Color color)
	{
	   Sprite sprite = new Sprite("Rectangle");
	   
	   SpriteCostume spriteCostume = SampleSpriteCostumes.createRectangleSpriteCostume(width, height, color);
	   spriteCostume.setXReferencePercent(0.0);
	   spriteCostume.setYReferencePercent(0.0);
	   sprite.addCostume(spriteCostume);
	      
	   return sprite;
	}

   public static Sprite createCrossHairs()
   {
      Sprite crossHairs = new Sprite("CrossHairs");
      crossHairs.addCostume(SampleSpriteCostumes.getCrossHairs());
      return crossHairs;
   }
   
   public static Sprite createCheckeredBall(String name)
   {
      Sprite ballSprite = new Sprite(name);
      SpriteCostume ballCostume = SpriteCostume.createFromFile("SampleImages/Balls/Checkerd-Ball-Arvin61r58-300px.png");

      ballSprite.addCostume(ballCostume);
      return ballSprite;
   }
   
   public static Sprite createSoccerBall(String name)
   {
      Sprite ballSprite = new Sprite(name);
      SpriteCostume ballCostume = SpriteCostume.createFromFile("SampleImages/Balls/Gioppino-Soccer-Ball-300px.png");
      
      ballSprite.addCostume(ballCostume);
      return ballSprite;
   }
   
   public static Sprite createBasketball(String name)
   {
      Sprite ballSprite = new Sprite(name);
      SpriteCostume ballCostume = SpriteCostume.createFromFile("SampleImages/Balls/krepsinio-kamuolys-300px.png");
      
      ballSprite.addCostume(ballCostume);
      return ballSprite;
   }
   
}
