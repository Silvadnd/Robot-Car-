#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h> // For abs()

// --- Pin Definitions ---
#define STBY_PIN GPIO_NUM_33
#define AIN1_PIN GPIO_NUM_25
#define AIN2_PIN GPIO_NUM_26
#define PWMA_PIN GPIO_NUM_27
#define BIN1_PIN GPIO_NUM_14
#define BIN2_PIN GPIO_NUM_13
#define PWMB_PIN GPIO_NUM_12
#define LEFT_IR_PIN GPIO_NUM_36
#define MIDDLE_IR_PIN GPIO_NUM_39
#define RIGHT_IR_PIN GPIO_NUM_34
#define LEFT_IR_CHANNEL ADC_CHANNEL_0
#define MIDDLE_IR_CHANNEL ADC_CHANNEL_3
#define RIGHT_IR_CHANNEL ADC_CHANNEL_6
#define TRIG_PIN GPIO_NUM_23
#define ECHO_PIN GPIO_NUM_19
#define SERVO_PIN GPIO_NUM_18
#define I2C_MASTER_SDA_IO GPIO_NUM_16
#define I2C_MASTER_SCL_IO GPIO_NUM_17

// --- Constants ---
#define MOTOR_BASE_SPEED 55 // Base speed set to 55
#define MAX_MOTOR_SPEED 255
#define MAX_LEDC_DUTY 8191
#define OBSTACLE_THRESHOLD 20 // Increased threshold (e.g., to 20cm) - CALIBRATE!
#define IR_THRESHOLD 800     // Adjust! Assuming LOWER means BLACK.
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_TIMEOUT_MS 1000
#define MPU6050_I2C_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_WHO_AM_I 0x75

// --- Constants for Obstacle Avoidance ---
#define TURN_DELAY_MS 400         // Turning duration
#define FORWARD_DELAY_MS 500      // Forward movement duration
#define MANEUVER_SPEED 60         // Speed during obstacle avoidance maneuvers

float Kp = 15.0; // Reduced Kp STARTING VALUE for re-tuning at speed 55. Adjust based on observation.
float Kd = 7.0; // !! MUST RE-TUNE for speed 55 !!

// --- Global Variables ---
float distance_F; // Keep front distance global
adc_oneshot_unit_handle_t adc1_handle;
static const char *TAG = "ROBOT_CAR";
int16_t accel_x = 0, accel_y = 0, accel_z = 0;
int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
int error = 0;
int previous_error = 0;

// --- Function Prototypes ---
void init_gpio();
void init_pwm();
void init_adc();
esp_err_t init_i2c();
esp_err_t init_mpu6050();
void servo_write(int angle);
float measure_distance();
int read_ir_sensor(adc_channel_t channel);
esp_err_t mpu6050_read_raw_data();
uint32_t scale_speed_to_duty(int speed);
void motor_control(int left_speed_cmd, bool left_dir, int right_speed_cmd, bool right_dir);
void motor_forward(int speed);
void motor_turn_right(int speed);
void motor_turn_left(int speed);
void motor_stop();
void check_sides_for_escape(float *distance_L_out, float *distance_R_out);
void avoid_obstacle();
void search_for_line();

// --- Initialization Functions ---
void init_gpio() {
    gpio_set_direction(AIN1_PIN, GPIO_MODE_OUTPUT); gpio_set_direction(AIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN1_PIN, GPIO_MODE_OUTPUT); gpio_set_direction(BIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(STBY_PIN, GPIO_MODE_OUTPUT); gpio_set_level(STBY_PIN, 0);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT); gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LEFT_IR_PIN, GPIO_MODE_INPUT); gpio_set_direction(MIDDLE_IR_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(RIGHT_IR_PIN, GPIO_MODE_INPUT);
    ESP_LOGI(TAG, "GPIO Initialized");
}

void init_pwm() {
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel_pwma = { .gpio_num = PWMA_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_pwma));
    ledc_channel_config_t ledc_channel_pwmb = { .gpio_num = PWMB_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_pwmb));
    ledc_channel_config_t ledc_channel_servo = { .gpio_num = SERVO_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_servo));
    ESP_LOGI(TAG, "PWM Initialized");
}

void init_adc() {
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LEFT_IR_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MIDDLE_IR_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, RIGHT_IR_CHANNEL, &config));
    ESP_LOGI(TAG, "ADC Initialized");
}

esp_err_t init_i2c() {
    i2c_config_t conf = { .mode = I2C_MODE_MASTER, .sda_io_num = I2C_MASTER_SDA_IO, .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = I2C_MASTER_FREQ_HZ, };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) { ESP_LOGE(TAG, "I2C parameter config error: %s", esp_err_to_name(err)); return err; }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "I2C driver install error: %s", esp_err_to_name(err)); return err; }
    ESP_LOGI(TAG, "I2C Initialized (SDA: %d, SCL: %d)", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return ESP_OK;
}

esp_err_t init_mpu6050() {
    uint8_t data = 0;
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR, &((uint8_t){MPU6050_WHO_AM_I}), 1, &data, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) { ESP_LOGE(TAG, "I2C read WHO_AM_I failed: %s", esp_err_to_name(err)); return err; }
    if (data != MPU6050_I2C_ADDR) { ESP_LOGE(TAG, "MPU6050 not found or wrong WHO_AM_I value: 0x%02X", data); return ESP_FAIL; }
    ESP_LOGI(TAG, "MPU6050 Found! WHO_AM_I = 0x%02X", data);
    uint8_t write_buf[2] = {MPU6050_PWR_MGMT_1, 0x00};
    err = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) { ESP_LOGE(TAG, "I2C write to PWR_MGMT_1 failed: %s", esp_err_to_name(err)); return err; }
    ESP_LOGI(TAG, "MPU6050 Initialized (Woken Up)"); vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

// --- Core Sensor/Actuator Functions ---
void servo_write(int angle) {
    int duty_us = (angle * 11) + 500;
    uint32_t duty = (uint32_t)((float)duty_us / 20000.0 * MAX_LEDC_DUTY);
    if (duty > MAX_LEDC_DUTY) { duty = MAX_LEDC_DUTY; }
    if (duty_us < 500) { duty = (uint32_t)(500.0 / 20000.0 * MAX_LEDC_DUTY); }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
}

float measure_distance() {
    gpio_set_level(TRIG_PIN, 0); esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1); esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);
    uint32_t start_time = 0, end_time = 0, timeout_us = 30000;
    uint32_t pulse_start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) { if ((esp_timer_get_time() - pulse_start_time) > timeout_us) { return 999.0; } }
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) { if ((esp_timer_get_time() - start_time) > timeout_us) { return 999.0; } }
    end_time = esp_timer_get_time();
    return ((float)(end_time - start_time) * 0.0343) / 2.0;
}

int read_ir_sensor(adc_channel_t channel) {
    int adc_raw;
    if (adc_oneshot_read(adc1_handle, channel, &adc_raw) != ESP_OK) { return -1; }
    return (adc_raw < IR_THRESHOLD) ? 1 : 0;
}

esp_err_t mpu6050_read_raw_data() {
    uint8_t data[14];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR,
                                       &((uint8_t){MPU6050_ACCEL_XOUT_H}), 1, data, 14, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data: %s", esp_err_to_name(err));
        accel_x = accel_y = accel_z = 0; gyro_x = gyro_y = gyro_z = 0;
        return err;
    }
    accel_x = (int16_t)((data[0] << 8) | data[1]); accel_y = (int16_t)((data[2] << 8) | data[3]); accel_z = (int16_t)((data[4] << 8) | data[5]);
    gyro_x = (int16_t)((data[8] << 8) | data[9]); gyro_y = (int16_t)((data[10] << 8) | data[11]); gyro_z = (int16_t)((data[12] << 8) | data[13]);
    return ESP_OK;
}

// --- Motor Control Functions ---
uint32_t scale_speed_to_duty(int speed) {
    if (speed < 0) { speed = 0; } if (speed > 255) { speed = 255; }
    return (uint32_t)((float)speed / 255.0 * MAX_LEDC_DUTY);
}

void motor_control(int left_speed_cmd, bool left_dir, int right_speed_cmd, bool right_dir) {
    int clamped_left_speed = abs(left_speed_cmd); int clamped_right_speed = abs(right_speed_cmd);
    if (clamped_left_speed > MAX_MOTOR_SPEED) { clamped_left_speed = MAX_MOTOR_SPEED; } if (clamped_right_speed > MAX_MOTOR_SPEED) { clamped_right_speed = MAX_MOTOR_SPEED; }
    gpio_set_level(AIN1_PIN, left_dir ? 1 : 0); gpio_set_level(AIN2_PIN, left_dir ? 0 : 1);
    uint32_t left_duty = scale_speed_to_duty(clamped_left_speed);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left_duty)); ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    gpio_set_level(BIN1_PIN, right_dir ? 1 : 0); gpio_set_level(BIN2_PIN, right_dir ? 0 : 1);
    uint32_t right_duty = scale_speed_to_duty(clamped_right_speed);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right_duty)); ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}
void motor_forward(int speed) { motor_control(speed, 1, speed, 1); }
void motor_turn_right(int speed) { motor_control(speed, 1, speed, 0); }
void motor_turn_left(int speed) { motor_control(speed, 0, speed, 1); }
void motor_stop() { motor_control(0, 1, 0, 1); }

// --- Obstacle Avoidance Logic ---

// Function to scan sides and return best escape direction
void check_sides_for_escape(float *distance_L_out, float *distance_R_out) {
    ESP_LOGI(TAG, "Checking sides for escape route...");
    motor_stop(); 
    vTaskDelay(pdMS_TO_TICKS(200)); // Ensure stopped

    // Look right
    ESP_LOGD(TAG, "Scanning Right...");
    servo_write(170);
    vTaskDelay(pdMS_TO_TICKS(500));
    *distance_R_out = measure_distance();
    ESP_LOGI(TAG, "Distance Right = %.1f cm", *distance_R_out);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Look left
    ESP_LOGD(TAG, "Scanning Left...");
    servo_write(10);
    vTaskDelay(pdMS_TO_TICKS(600)); // Allow extra time for servo travel
    *distance_L_out = measure_distance();
    ESP_LOGI(TAG, "Distance Left = %.1f cm", *distance_L_out);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Center Servo
    ESP_LOGD(TAG, "Centering Servo...");
    servo_write(90); 
    vTaskDelay(pdMS_TO_TICKS(300));
}

// Arduino-style obstacle avoidance implementation
void avoid_obstacle() {
    ESP_LOGI(TAG, "Obstacle Detected! Distance: %.1f cm. Starting avoidance.", distance_F);
    error = 0; 
    previous_error = 0; // Reset PD error
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check sides
    float local_distance_L, local_distance_R;
    check_sides_for_escape(&local_distance_L, &local_distance_R);
    
    // Compare distances and execute avoidance maneuver
    if (local_distance_L > local_distance_R && local_distance_L > OBSTACLE_THRESHOLD) {
        // More space to the left
        ESP_LOGI(TAG, "Navigating around obstacle via left");
        
        // Turn left
        motor_turn_left(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        
        // Forward
        motor_forward(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_DELAY_MS));
        
        // Turn right
        motor_turn_right(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        
        // Forward
        motor_forward(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_DELAY_MS + 100)); // Slightly longer
        
        // Turn right again
        motor_turn_right(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        
        // Forward
        motor_forward(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_DELAY_MS));
        
        // Final turn left to realign
        motor_turn_left(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS - 100)); // Slightly shorter
    } 
    else if (local_distance_R >= local_distance_L && local_distance_R > OBSTACLE_THRESHOLD) {
        // More space to the right
        ESP_LOGI(TAG, "Navigating around obstacle via right");
        
        // Turn right
        motor_turn_right(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        
        // Forward
        motor_forward(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_DELAY_MS));
        
        // Turn left
        motor_turn_left(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        
        // Forward
        motor_forward(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_DELAY_MS + 100)); // Slightly longer
        
        // Turn left again
        motor_turn_left(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        
        // Forward
        motor_forward(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_DELAY_MS));
        
        // Final turn right to realign
        motor_turn_right(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS - 100)); // Slightly shorter
    }
    else {
        // Not enough space either way, back up and turn around
        ESP_LOGW(TAG, "Not enough space in either direction. Turning around.");
        motor_control(MANEUVER_SPEED, 0, MANEUVER_SPEED, 0); // Backward
        vTaskDelay(pdMS_TO_TICKS(800));
        motor_turn_left(MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(1400));  // 180 degree turn
    }
    
    // Stop motors and prepare to resume line following
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Obstacle avoidance complete, resuming main loop.");
    
    // Ensure servo is centered before returning
    servo_write(90);
    vTaskDelay(pdMS_TO_TICKS(200));
}

// Function to search for line if lost during obstacle avoidance
void search_for_line() {
    ESP_LOGI(TAG, "Searching for line...");
    static int search_direction = 1;  // 1 = right, -1 = left
    static int search_steps = 0;
    
    // Switch search direction after several steps
    if (search_steps > 20) {
        search_direction *= -1;
        search_steps = 0;
    }
    
    if (search_direction > 0) {
        // Search right
        motor_turn_right(45);
        vTaskDelay(pdMS_TO_TICKS(100));
        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(50));
    } else {
        // Search left
        motor_turn_left(45);
        vTaskDelay(pdMS_TO_TICKS(100));
        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    search_steps++;
}

// --- Main Application ---
void app_main(void) {
    ESP_LOGI(TAG, "Robot Car Starting (PD Control - Arduino-Style Avoidance)..."); // Updated version tag
    init_gpio(); init_pwm(); init_adc();
    if (init_i2c() != ESP_OK) { ESP_LOGE(TAG, "I2C init failed. Halting."); while(1) vTaskDelay(portMAX_DELAY); }
    if (init_mpu6050() != ESP_OK) { ESP_LOGE(TAG, "MPU6050 init failed."); }
    gpio_set_level(STBY_PIN, 1); ESP_LOGI(TAG, "Motor Driver Enabled (STBY HIGH)");
    servo_write(90); vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t loop_counter = 0;
    while (1) {
        loop_counter++;
        distance_F = measure_distance();
        int ir_L = read_ir_sensor(LEFT_IR_CHANNEL); int ir_M = read_ir_sensor(MIDDLE_IR_CHANNEL); int ir_R = read_ir_sensor(RIGHT_IR_CHANNEL);
        if (mpu6050_read_raw_data() != ESP_OK) { ESP_LOGW(TAG, "MPU6050 read failed this cycle."); }

        if (loop_counter % 20 == 0) { // Periodic logging
            ESP_LOGI(TAG, "Sensors: DistF=%.1f IR L%d M%d R%d | Err:%d | Accel X%d Y%d Z%d | Gyro X%d Y%d Z%d",
                     distance_F, ir_L, ir_M, ir_R, error, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        }

        // --- Priority 1: Obstacle Avoidance (Arduino-style) ---
        if (distance_F < OBSTACLE_THRESHOLD && distance_F > 0.1) { // Added > 0.1 check to avoid false trigger on bad readings
            avoid_obstacle(); // Using the Arduino-style obstacle avoidance function
            continue; // Skip line following for this iteration
        }

        // --- Priority 2: Line Following (PD Control) ---
        bool line_lost = false;
        if (ir_L == 0 && ir_M == 1 && ir_R == 0) { error = 0; }
        else if (ir_L == 0 && ir_M == 1 && ir_R == 1) { error = 1; }
        else if (ir_L == 1 && ir_M == 1 && ir_R == 0) { error = -1; }
        else if (ir_L == 0 && ir_M == 0 && ir_R == 1) { error = 2; }
        else if (ir_L == 1 && ir_M == 0 && ir_R == 0) { error = -2; }
        else if ((ir_L == 0 && ir_M == 0 && ir_R == 0) || (ir_L == 1 && ir_M == 1 && ir_R == 1)) {
             line_lost = true; error = previous_error * 2;
             if (error > 2) { error = 2; } if (error < -2) { error = -2; }
             ESP_LOGW(TAG, "Line Lost/Intersection! Amplified previous error: %d", error);
        } else { line_lost = true; error = previous_error; ESP_LOGW(TAG, "Ambiguous IR (%d%d%d)! Using previous error: %d", ir_L, ir_M, ir_R, error); }

        if (line_lost && error == 0) {
            ESP_LOGW(TAG, "Line Lost and Previous Error 0 - Searching for line");
            search_for_line(); // Use our new search function instead of stopping
        } else {
            int p_term = Kp * error; int d_term = Kd * (error - previous_error);
            int correction = p_term + d_term;
            int left_speed_cmd = MOTOR_BASE_SPEED - correction;
            int right_speed_cmd = MOTOR_BASE_SPEED + correction;
            bool left_dir = (left_speed_cmd >= 0); bool right_dir = (right_speed_cmd >= 0);
            motor_control(left_speed_cmd, left_dir, right_speed_cmd, right_dir);

            if (loop_counter % 5 == 0) {
                 ESP_LOGD(TAG, "PD: Err=%d, PrevErr=%d, P=%d, D=%d, Corr=%d | LSpd=%d(%d), RSpd=%d(%d)",
                          error, previous_error, p_term, d_term, correction, left_speed_cmd, left_dir, right_speed_cmd, right_dir);
            }
        }
        previous_error = error;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}