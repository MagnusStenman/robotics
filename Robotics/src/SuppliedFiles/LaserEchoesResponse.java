package SuppliedFiles;

import java.util.ArrayList;
import java.util.Map;


public class LaserEchoesResponse implements Response
{
   private Map<String, Object> data;

   public void setData(Map<String, Object> data)
   {
      this.data = data;
   }

   public double[] getEchoes()
   {
      ArrayList echoes = (ArrayList)data.get("Echoes");
      
      Object[] list = echoes.toArray();
      double[] result = new double[list.length];
      for (int i= 0 ; i < result.length; i++)
         // Unfortunately the JSON decoder looks at the value and converts it
         // to either a double if there is a decimal point or an int otherwise
         if (list[i] instanceof Double)
            result[i] = ((Double)list[i]).doubleValue();
         else
            result[i] = ((Integer)list[i]).intValue();
      
      return result;
   }

   public String getPath()
   {
      return "/lokarria/laser/echoes";
   }

   public long getTimestamp()
   {
      return (Long)data.get("TimeStamp");
   }

}
