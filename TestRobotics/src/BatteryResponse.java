import java.util.Map;

public class BatteryResponse implements Response
{

   private Map<String, Object> data;

   public void setData(Map<String, Object> data)
   {
      this.data = data;
   }

   public double getRemaining()
   {
      
//      System.out.println(data.get("Remaining"));
//      
//      return 0;
      return ((Integer)data.get("Remaining")).doubleValue();
   }

   public double getStatus()
   {
      return Double.parseDouble( (String)data.get("Status") );
   }

   public String getPath()
   {
      return "/lokarria/battery";
   }

   public long getTimestamp()
   {
      return (Long)data.get("TimeStamp");
   }


}
