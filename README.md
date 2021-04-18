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
static unsigned gSendDate = 0 ;
static unsigned gSentCount = 0 ;
static unsigned gReceivedCount = 0 ;

void loop () {
  CANMessage message ;
  if (gSendDate < millis ()) {
    message.id = 0x542 ;
    const bool ok = ACAN_ESP32::can.tryToSend (message) ;
    if (ok) {
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentCount) ;
    }
  }
  if (ACAN_ESP32::can.receive (message)) {
    gReceivedCount += 1 ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
  }
}
```
`CANMessage` is the class that defines a CAN message. The `message` object is fully initialized by the default constructor. Here, we set the `id` to `0x542` for sending a standard data frame, without data, with this identifier.

The `ACAN_ESP32::can.tryToSend` tries to send the message. It returns `true` if the message has been sucessfully added to the driver transmit buffer.

The `gSendDate` variable handles sending a CAN message every 2000 ms.

`ACAN_ESP32::can.receive` returns `true` if a message has been received, and assigned to the `message`argument.

### Use of Optional Reception Filtering

The hardware defines two kinds of filters: *primary* and *secondary* filters. Depending the driver configuration, you can have up to 14 *primary* filters and 18 *secondary* filters.

This an setup example:

```cpp
  ACAN_ESP32_Settings settings (125 * 1000) ;
  ...
   const ACANPrimaryFilter primaryFilters [] = {
    ACANPrimaryFilter (kData, kExtended, 0x123456, handle_myMessage_0)
  } ;
  const ACANSecondaryFilter secondaryFilters [] = {
    ACANSecondaryFilter (kData, kStandard, 0x234, handle_myMessage_1),
    ACANSecondaryFilter (kRemote, kStandard, 0x542, handle_myMessage_2)
  } ;
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings,
                                               primaryFilters, 
                                               1, // Primary filter array size
                                               secondaryFilters,
                                               2) ; // Secondary filter array size
```
For example, the first filter catches extended data frames, with an identifier equal to `0x123456`. When a such frame is received, the `handle_myMessage_0` function is called. In order to achieve this by-filter dispatching, you should call `ACAN_ESP32::can.dispatchReceivedMessage` instead of `ACAN_ESP32::can.receive` in the `loop`function:


```cpp
void loop () {
  ACAN_ESP32::can.dispatchReceivedMessage () ; // Do not use ACAN_ESP32::can.receive any more
  ...
}
```
