#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define _DUTY_MIN 630 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1180 // servo neutral position (90 degree)
#define _DUTY_MAX 2300 // servo full counterclockwise position (180 degree)

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define _SERVO_SPEED 100 // servo speed limit (unit: degree/second)
#define INTERVAL 180  // servo update interval

#define ALPHA 0.01  

// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr;
float dist_raw, dist_cali, dist_ema, alpha;
int a, b;


void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);
  
// initialize serial port
  Serial.begin(115200);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) / 180 * _SERVO_SPEED * INTERVAL / 1000;

// initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  a = 65;
  b = 210;
  alpha = ALPHA;
  
// initialize last sampling time
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  dist_raw = ir_distance();
  dist_ema = alpha * dist_raw + (1 - alpha) * dist_ema;
  dist_cali = 100 + 300.0 / (b - a) * (dist_ema - a);

  if (dist_cali < 255) {
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  else {
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(dist_ema);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
}
