## CAN Library for ESP32

### ACAN_ESP32 library description
ACAN_ESP32 is a driver for the CAN module built into the ESP32 microcontroller.

The driver supports many bit rates, as standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, and 1 Mbit/s. An efficient CAN bit timing calculator finds settings for them, but also for an exotic bit rate as 842 kbit/s. If the wished bit rate cannot be achieved, the `begin` method does not configure the hardware and returns an error code.

> The library is in the `ACAN_ESP32` directory. Driver API is fully described by the PDF file in the `ACAN_ESP32/extras` directory.

### Demo Sketch

> The demo sketch is in the `ACAN_ESP32/examples/LoopBackDemo` directory.

> Note the driver required name is `ACAN_ESP32::can`; you cannot use an other name. 

Configuration is a four-step operation.

1. Instanciation of the `settings` object : the constructor has one parameter: the wished CAN bit rate. The `settings` is fully initialized.
2. You can override default settings. Here, we set the `mRequestedCANMode` property to `ACAN_ESP32_Settings::LoopBackMode`, enabling to run self reception of sent frame. **Unlike other microcontrollers, this mode requires the connection of a transceiver.** We can also for example change the receive buffer size by setting the `mReceiveBufferSize` property.
3. Calling the `begin` method configures the driver and starts CAN bus participation. Any message can be sent, any frame on the bus is received. No default filter to provide.
4. You check the `errorCode` value to detect configuration error(s).

```cpp
void setup () {
  Serial.begin (9600) ;
  Serial.println ("Hello") ;
  ACAN_ESP32_Settings settings (125 * 1000) ; // 125 kbit/s
  settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings) ;
  if (0 == errorCode) {
    Serial.println ("Can ok") ;
  }else{
    Serial.print ("Error Can: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}
```

Now, an example of the `loop` function. As we have selected loop back mode, every sent frame is received.

```cpp
static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

void loop () {
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 500 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    const bool ok = ACAN_ESP32::can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  }
  while (ACAN_ESP32::can.receive (frame)) {
    gReceivedFrameCount += 1 ;
  }
}
```
`CANMessage` is the class that defines a CAN message. The `message` object is fully initialized by the default constructor. Here, we set the `id` to `0x542` for sending a standard data frame, without data, with this identifier.

The `ACAN_ESP32::can.tryToSend` tries to send the message. It returns `true` if the message has been sucessfully added to the driver transmit buffer.

The `gBlinkLedDate ` variable handles sending a CAN message every 500 ms.

`ACAN_ESP32::can.receive` returns `true` if a message has been received, and assigned to the `message`argument.

