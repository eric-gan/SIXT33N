/*
 * turning.ino
 *
 * Implementing turns in SIXT33N
 *
 * Eric Gan, Shloak Jain, Arjun Mishra
 */

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*    From closed_loop.ino   */
/*        with changes       */
/*---------------------------*/

// playing around with changing v*

float theta_left = 0.5704;
float theta_right = 0.5996;
float beta_left = 32.04;
float beta_right = 25.31;
float v_star = (29.1 * 1.5) / 5.0;

// PWM inputs to jolt the car straight
int left_jolt = 240;
int right_jolt = 200;

// Control gains
float k_left = 0.9;
float k_right = 1.05;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*    From closed_loop.ino   */
/*---------------------------*/

float driveStraight_left_OL(float v_star) {
  return (v_star + beta_left) / theta_left;
}

float driveStraight_right_OL(float v_star) {
  return (v_star + beta_right) / theta_right;
}


float driveStraight_left(float v_star, float delta) {
  return driveStraight_left_OL(v_star) - (k_left / theta_left) * delta;
}

float driveStraight_right(float v_star, float delta) {
      return driveStraight_right_OL(v_star) + (k_right / theta_right) * delta;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*    From closed_loop.ino   */
/*---------------------------*/

float delta_ss = -4;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 5000, 2500, 5000};

float delta_reference(int k) {
  // YOUR CODE HERE
  if (drive_mode == DRIVE_RIGHT) {
    return -v_star*CAR_WIDTH*k / TURN_RADIUS;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return v_star*CAR_WIDTH*k / TURN_RADIUS;;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  for (int i = 0; i <= 4; i++) {
    sample_lens[i] = run_times[i]/SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  setTimer(); // Set timer for timestep
  start_drive_mode();
}

void loop(void) {
  check_encoders();
  if (loop_mode == MODE_LISTEN) {
    // In the integration phase of the project, this section will listen
    // to the microphone and switch to the specified mode.
    // For now, we simply cycle through them.
    drive_mode = (drive_mode + 1) % 4;

    start_drive_mode();
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(v_star*5, delta);
      int right_cur_pwm = driveStraight_right(v_star*5, delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
}

void start_listen_mode(void) {
  write_pwm(0, 0);
  delay(3000);
  loop_mode = MODE_LISTEN;
}

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
  TA2CCTL0 = CCIE; // enable interrupts for Timer A
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (loop_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
