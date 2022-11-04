import java.util.*;
public class ArrayListUtilities {
   public static void main(String[] args) {
      ArrayList<Integer> nums = new ArrayList<Integer>();
      nums.add(27);
      seed(nums, 10, 1, 10);
      System.out.println(nums);
      sortIAL(nums);
      System.out.println(nums);
   }
   
   public static void seed(ArrayList<Integer> list, int numElement, int minValue, int maxValue) {
      int range = maxValue - minValue + 1;
      for (int i = 0; i < numElement; i++) {
         int rand = (int)(Math.random() * range) + minValue;
         list.add(rand);
      }
   }
   
   public static int max(ArrayList<Integer> list) {
      int max = list.get(0);
      for (int i = 0; i < list.size(); i++) {
         if (list.get(i) > max) {
            max = list.get(i);
         }
      }
      return max;
   }
   
   public static int min(ArrayList<Integer> list) {
      int min = list.get(0);
      for (int i = 0; i < list.size(); i++) {
         if (list.get(i) < min) {
            min = list.get(i);
         }
      }
      return min;
   }
   
   public static void sortIAL(ArrayList<Integer> list) {
      ArrayList<Integer> temp = new ArrayList<Integer>();
      int listSize = list.size();
      while (temp.size() != listSize) {
         temp.add(min(list));
         list.remove(list.indexOf(min(list)));
      }
      list.clear();
      for (int i = 0; i < temp.size(); i++) {
         list.add(temp.get(i));
      }
   }
}