package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.HashMap;
import java.util.Map;

public class LookupTable {
    public static void main(String[] args) {
        Map<String, Double> lookupTable = new HashMap<>();
        lookupTable.put("Uno", 1.0);
        lookupTable.put("Dos", 2.0);
        lookupTable.put("Tres", 3.0);
        lookupTable.get(1.5);

    System.out.println("Value for 'Uno': " + lookupTable.get("Uno"));
       


    }



}
