# NibeF730-to-MQTT
This is a Nibe GW for ESP32 which publish all data to MQTT.
It is based on [openhab](https://github.com/openhab/openhab-addons/tree/8ea489838e89f8f54a1e05260e3801222a71817d/bundles/org.openhab.binding.nibeheatpump/contrib/NibeGW/Arduino/NibeGW).

To me important registers are known to NibeGW so it can interpret the values.  
Nibe will send out a set of 20 register values, which are defined by the LOG.SET file per USB on the heat pump.  
Additional registers can be read by the topic `nibeF730/read/40004`, every known register can be chosen after read.  
For editing register values of the heat pump publish the value to `nibeF730/write/43005 -60`, the logic of read applies here too.  
If you need more registers you can add them through publishing to this topic `nibeF730/add {"register":"43005","factor":"10","size":"4"}`, the size parameter is defined as follows:  
`0:=unsigned 1 byte(U8); 1:=unsigned 2 bytes(U16); 2:=unsigned 4 bytes(U32); 3:=signed 1 byte(S8); 4:=signed 2 bytes(S16); 5:=signed 4 bytes(S32)`.  
The factor is used to transform decimals into integers, like this `nibeF730/write/43005 -12.7`, the -12.7 is multiplied with 10 to form -127 which is send to the heat pump or the other way round, the heat pump sends -778 and it got divided by 10 to form -77.8.
