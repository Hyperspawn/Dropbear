# QDD Actuator Torque Calculation

## Overview
This document outlines the torque calculation for a **Quasi-Direct Drive (QDD) Actuator** based on experimental measurements.

## Calculation Details

### **Given Data**
- **Length of link:** `0.37m` (from center)
- **Mass on weighing scale:** `2.4 kg`
- **Gravitational acceleration:** `g = 9.81 m/s²`

### **Force Calculation**
Force due to gravity is given by:

```
F = m × g
```

Substituting the values:

```
F = 2.4 × 9.81
F = 23.54 N
```

### **Torque Calculation**
Torque is calculated using:

```
T = F × r
```

Substituting the values:

```
T = 23.54 × 0.37
T = 8.709 N·m
```

### **Final Result:**
**Torque (T) = `8.709 N·m`**

---

## **Test Conditions**
- **Voltage applied:** `31V`
- **Current drawn:** `10A`

These values represent the conditions under which the actuator was tested.

---

## **Notes**
- The torque was calculated based on a static weight measurement.
- Dynamic effects, back-EMF, and motor efficiency were not considered in this calculation.

---

## **Contributing**
If you find any errors or have suggestions for improvements, feel free to open a pull request!

---
