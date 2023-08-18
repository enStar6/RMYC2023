
// ---------------------------------------------------------------------------
//翻译：这个例子展示了如何使用NewPing的ping_timer方法，该方法使用Timer2中断来获取
 
//平时间。与标准ping方法相比，使用此方法的优点是，它允许更高的精度
 
//事件驱动的草图，让你看起来同时做两件事。一个例子就是ping
 
//一个超声波传感器，用于在导航的同时防止可能发生的碰撞。这使得
 
//正确绘制草图以完成多任务。请注意，由于ping_timer方法使用Timer2，
 
//其他也使用Timer2的功能或库也会受到影响。例如，PWM功能开启
 
//Arduino Uno上的插脚3和11（Arduino Mega上的插脚9和10）以及音调库。注意，只有PWM
 
//引脚功能丢失（因为它们使用Timer2进行PWM），引脚仍可使用。
 
//注：对于Teensy/Leonardo（ATmega32U4），该库使用Timer4而不是Timer2。
// ---------------------------------------------------------------------------
#include <NewPing.h>
 
#define TRIGGER_PIN   12 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN      11 // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
 
unsigned int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.
 
void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pingTimer = millis(); // Start now.
}
 
void loop() {
  // Notice how there's no delays in this sketch to allow you to do other processing in-line while doing distance pings.
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  }
//  Serial.println("hello");
  // Do other stuff here, really. Think of it as multi-tasking.
}
void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // Here's where you can add code.
//    Serial.print("Ping: ");
    Serial.println(sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
//为了方便串口绘图，此处同样进行了修改
//    Serial.println("cm");
  }
  // Don't do anything here!

}