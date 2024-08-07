# Distance Measurment By Ultrasonic Sensor

## How Ultrasonic Sensors Work

Ultrasonic sensors, commonly used for distance measurement, operate using high-frequency sound waves (beyond the range of human hearing). The operation of the sensor is as follows:

1. **Generating an Ultrasonic Pulse:**
   - The ultrasonic sensor has a transducer that can generate high-frequency sound waves. This transducer is briefly "triggered" or "excited" to send an ultrasonic pulse towards the target.
   - The pulse travels towards the target object and disperses in the environment.

2. **Scattering and Reflecting the Pulse:**
   - The sound pulse travels to the target object and, when it hits the object, part of it is reflected back towards the sensor.

3. **Receiving the Reflected Pulse:**
   - The sensor has another transducer designed to receive the reflected pulse. This transducer captures the reflected pulse and converts it into an electrical signal.

4. **Calculating the Time of Flight:**
   - The sensor measures the precise time it takes for the pulse to travel from sending to receiving. This time is usually measured in microseconds (µs).

5. **Calculating Distance:**
   - Using the time of flight and the speed of sound in air (approximately 343 meters per second at room temperature), the distance between the sensor and the target object can be calculated.
   - The distance calculation formula is:
     \[
     \text{Distance} = \frac{\text{Time of Flight} \times \text{Speed of Sound}}{2}
     \]
   - The division by 2 accounts for the round trip of the pulse.

### Roles of TRIG and ECHO Pins

- **TRIG (Trigger) Pin:**
  - This pin is used to send the ultrasonic pulse. To start the measurement, the TRIG pin should be set to HIGH (high) for a short duration (usually about 10 microseconds). This action causes the sensor to emit an ultrasonic pulse.
  - After the pulse is sent, the TRIG pin should be set back to LOW (low) to prepare the sensor for receiving the reflected signal.

- **ECHO Pin:**
  - The ECHO pin is used to receive the reflected pulses. When the ultrasonic pulse hits the target and reflects back, the ECHO pin receives the reflected signal.
  - The duration for which the ECHO pin remains HIGH indicates the time taken for the pulse to travel to the target and back. By measuring this time and using the speed of sound, the distance can be accurately calculated.

## Configuration STM32

The timer must be configured to operate in Input Capture mode, and the Counter Period should be set to 0xFFFF. Then, the timer's Prescaler should be configured so that the timer clock is set to 1 MHz. This means that 1 microsecond elapses for each clock pulse.

### `void delay(uint16_t time)`

This function provides a delay in microseconds using the TIM1 hardware timer.

**Parameters:**
- `time` (uint16_t): The delay duration in milliseconds.

**Description:**
- The `delay` function utilizes the TIM1 timer to create a precise delay. It sets the TIM1 counter to zero and waits until the counter value reaches the specified duration (`time`). This function effectively pauses the execution of the program for the desired time period.

**Usage:**
```c
void delay(uint16_t time) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);   // Reset timer counter
    while (__HAL_TIM_GET_COUNTER(&htim1) < time); // Wait until the counter reaches the specified time
}
```

###`void HYSRF05_Read(void)`

This function triggers the ultrasonic sensor (HYSRF05) to measure distance and starts the timer for capturing the echo pulse width.

**Description:**
- The `HYSRF05_Read` function generates a trigger pulse by setting the TRIG pin of the ultrasonic sensor high for a short duration and then setting it low. This pulse causes the sensor to emit an ultrasonic pulse.
- After triggering the sensor, the function enables the input capture interrupt on TIM1 to measure the duration of the echo pulse that returns from the obstacle.

**Usage:**
```c
void HYSRF05_Read(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  // Set TRIG pin high
    delay(10);  // Wait for 10 microseconds
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // Set TRIG pin low

    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);  // Enable capture interrupt
}
```

### `void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)`

This callback function is triggered when a capture event occurs on TIM1, used for measuring the duration of the echo pulse from the ultrasonic sensor.

**Parameters:**
- `htim` (TIM_HandleTypeDef*): Pointer to the TIM handle structure.

**Description:**
- The `HAL_TIM_IC_CaptureCallback` function handles input capture events from TIM1. It measures the time difference between two captured edge events on channel 1. This time difference represents the duration of the echo pulse.
- The function calculates the distance to the obstacle based on the pulse width and converts it to centimeters using the speed of sound.
- The callback toggles the capture polarity between rising and falling edges to measure the full duration of the echo pulse.

**Usage:**
```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (Is_First_Captured == 0) {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // First edge captured
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);  // Set to capture falling edge
        } else if (Is_First_Captured == 1) {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Second edge captured
            __HAL_TIM_SET_COUNTER(htim, 0);  // Reset counter

            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;  // Calculate pulse width
            } else {
                Difference = (0xffff - IC_Val1) + IC_Val2;  // Handle counter overflow
            }

            Distance = Difference * 0.0343 / 2;  // Calculate distance in cm
            Is_First_Captured = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);  // Set to capture rising edge
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);  // Disable capture interrupt
        }
    }
}
```