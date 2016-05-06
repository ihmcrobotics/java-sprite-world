package us.ihmc.javaSpriteWorld;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.WritableImage;

public class SampleSpriteCostumes
{
   public static SpriteCostume getWhiteChessKing()
   {
      return SpriteCostume.createFromFile("SampleImages/portablejim-Chess-tile-King-300px.png");
   }
   
   public static SpriteCostume getCrossHairs()
   {
      SpriteCostume crossHairs = SpriteCostume.createFromFile("SampleImages/Crosshairs-3456-300px.png");
      crossHairs.setXReferencePercent(0.50667);
      crossHairs.setYReferencePercent(0.50667);
      return crossHairs;
   }

   public static SpriteCostume createRectangleSpriteCostume(double width, double height, Color color)
   {
      int widthPixels = 100, heightPixels = 100;
      
      if (width > height)
      {
         widthPixels = (int) ((double) heightPixels * width / height);
      }
      else
      {
         heightPixels = (int) ((double) widthPixels * height / width);
      }
      
      BufferedImage bufferedImage = new BufferedImage(widthPixels, heightPixels, BufferedImage.TYPE_INT_ARGB);
      Graphics2D graphics2D = bufferedImage.createGraphics();

      System.out.println("widthPixels = " + widthPixels);
      System.out.println("heightPixels = " + heightPixels);
      graphics2D.setColor(color);
      graphics2D.fillRect(0, 0, widthPixels, heightPixels);
      
      graphics2D.setColor(Color.BLACK);
      graphics2D.drawRect(0, 0, widthPixels-1, heightPixels-1);
      
      graphics2D.dispose();

      WritableImage fxImage = SwingFXUtils.toFXImage(bufferedImage, null);
      SpriteCostume costume = new SpriteCostume(fxImage);
      costume.setXReferencePercent(0.0);
      costume.setYReferencePercent(0.0);

      return costume;
   }
}
