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
         result[i] = (Double)list[i];    // unboxing
      
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
