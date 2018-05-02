package us.ihmc.javaSpriteWorld;

import java.io.IOException;
import java.io.InputStream;
import java.net.URL;

import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

public class SpriteCostume
{
   private Image image;
   private Node javaFXNode;

   private double xReferencePercent, yReferencePercent;
   private double heightToWidthRatio;

   public static SpriteCostume createFromFile(String filename)
   {
      InputStream inputStream = SpriteCostume.class.getResourceAsStream(filename);
//      InputStream inputStream = ClassLoader.getSystemResourceAsStream(filename);
      if (inputStream == null)
      {
         System.err.println("Couldn't open resource as stream: " + filename);
         return null;
      }
      return createFromInputStream(inputStream);
   }

   public static SpriteCostume createFromURL(URL url)
   {
      InputStream inputStream;
      try
      {
         inputStream = url.openStream();
      }
      catch (IOException e)
      {
         System.err.println("Could not create inputStream from URL " + url);
         return null;
      }

      return createFromInputStream(inputStream);
   }

   public static SpriteCostume createFromInputStream(InputStream inputStream)
   {
      Image image = new Image(inputStream);
      SpriteCostume costume = new SpriteCostume(image);
      return costume;
   }

   public SpriteCostume(Image image)
   {
      this.image = image;

      xReferencePercent = 0.5;
      yReferencePercent = 0.5;

      setImage(image);
   }

   private void setImage(Image image)
   {
      if (image != null)
      {
    	  this.heightToWidthRatio = image.getHeight() / image.getWidth();
      }
      else
      {
    	  this.heightToWidthRatio = 1.0;
      }

      ImageView imageView = new ImageView(image);
      javaFXNode = imageView;
   }

   public Image getImage()
   {
      return image;
   }

   public double getImageWidthPixels()
   {
      return image.getWidth();
   }

   public double getImageHeightPixels()
   {
      return image.getHeight();
   }

   public double getHeightToWidthRatio()
   {
      return heightToWidthRatio;
   }

   public double getXReferencePercent()
   {
      return xReferencePercent;
   }

   public double getYReferencePercent()
   {
      return yReferencePercent;
   }

   public void setXReferencePercent(double xReferencePercent)
   {
      this.xReferencePercent = xReferencePercent;
   }

   public void setYReferencePercent(double yReferencePercent)
   {
      this.yReferencePercent = yReferencePercent;
   }

   public double getImageCenterX()
   {
      return image.getWidth() * xReferencePercent;
   }

   public double getImageCenterY()
   {
      return image.getHeight() * yReferencePercent;
   }

   public Node getJavaFXNode()
   {
      return javaFXNode;
   }
}
