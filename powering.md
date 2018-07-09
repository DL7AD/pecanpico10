# Powering

The Pecan can be either powered by following power sources. The power consumption is around 50mW in sleep mode and around 200mW at normal operation. If the transmitter is constantly transmitting, the tracker consumes around 500mW.

There are two power inputs which can be accessed from the 100mil-pinheader and one by USB. The solar cells must be connected to pin 1 and 2. The battery is connected on the backside to the BATT_CONN and GND pad. While the battery is physically connected to the circuit board, it's not directly connected to the circuit itself unless a jumper is connected between pin 3 and 4. Alternatively the battery can be connected to pin 4 and 15.

## Battery disconnect by USB

The circuit can also be powered externally by USB. In this case, the battery is disconnected from the circuit by Q1 and the circuit is only powered by USB. This means, that the battery can also not be charged by the solar cell. This particular design has been implemented to be used on a rocket. The circuit should be powered by the carrier rocket until the payload is being ejected in order to have full battery capacitance at the ejection event.

Note: Due to an error in the design of the Pecan Pico 10a, the battery must not be connected when USB is applied. You may connect USB and battery at the same time, if D1 and Q1 are removed. This bug has been fixed in the 10b variant and can be used as intended now.

## USB
USB can be used for operation at the computer. You may connect a battery to the battery input. Once USB is connected, the battery gets disconnected and the tracker is only powered over USB. Once USB is disconnected, the tracker is powered from the battery again. This feature is archived by a MOSFET and doesn not need any software cooperation.

## 3x Alkaline or Lithium Batteries
If you have only a time-limited operation planned, you might want to use primary batteries. The tracker needs at least 1.8V in order to operate. The camera needs at least 3.3V. So the tracker may be powered by three AA or AAA serial connected batteries from the +VBAT input. Note: You must not attach a solar cell while the tracker would try to charge it.

## A signle LiPO cell and solar cells
If a long operation is required, the tracker can be powered by a LiPO or LiIon battery. The battery can be charged by solar cells from the +VSOL input. The tracker may also be operated with solar cells only.


TODO: Power scheme missing
