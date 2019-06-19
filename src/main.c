/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <led.h>
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "arm_math.h"
#include "math.h"
#include "timer.h"
#include "asp.h"

#include "mpu6050.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define LOOP_RATE 1000	//200HZ 循环周期
#define GYRO_DPS	65536/1000.0	//1000量程 正负500
#define GYRO_CAL 1.0/LOOP_RATE/GYRO_DPS
#define ANGLE_CAL GYRO_CAL * (PI / 180)


int
main(int argc, char* argv[])
{
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  timer_start();

  blink_led_init();
  
  blink_led_off();

  init_mpu();

  MX_USB_DEVICE_Init();

  long acc_total_vector;

  // Infinite loop
  while (1)
    {
	  mpu_fetchdata();		//5ms 一次

	  raw_gyro = filter_gyro;		//使用滤波后对数据
	  raw_acc = filter_acc;

	  //可以开始使用陀螺仪的数据进行角度计算了(200HZ) 65.536 = 1d/s
	  angle_pitch += raw_gyro.y * GYRO_CAL;		//浮点数--> 发送到上位机后要转化为整数，扩大1000倍后发送
	  angle_roll  += raw_gyro.x * GYRO_CAL;

	  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
	  angle_pitch -= angle_roll * sin(raw_gyro.z * ANGLE_CAL);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
	  angle_roll  += angle_pitch * sin(raw_gyro.z * ANGLE_CAL);

	  //Accelerometer angle calculations
	  acc_total_vector = sqrt((raw_acc.x*raw_acc.x)+(raw_acc.y*raw_acc.y)+(raw_acc.z*raw_acc.z));       //Calculate the total accelerometer vector.

	  if(abs(raw_acc.y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
	    angle_pitch_acc = asin((float)raw_acc.y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
	  }
	  if(abs(raw_acc.x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
	    angle_roll_acc = asin((float)raw_acc.x/acc_total_vector)* -57.296;          //Calculate the roll angle.
	  }

	  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
	  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
	  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.

	  /**
	   * 0.1 这个取值非常重要，表示对加速度对信任，如果太小对话，会导致获取到对角度值变化非常慢。
	   *
	   * 一阶互补滤波
	   */
	  angle_pitch = angle_pitch * 0.900 + angle_pitch_acc * 0.100;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
	  angle_roll = angle_roll * 0.900 + angle_roll_acc * 0.100;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

	 // trace_printf("pitch = %d, roll=%d\n", (int32_t)(angle_pitch*1000),(int32_t)(angle_roll*1000));

	  data_exchange(SENSOR);

      HAL_Delay(1);

//      blink_led_on();
//      timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);
//
//      blink_led_off();
//      timer_sleep(BLINK_OFF_TICKS);
//
//      ++seconds;
//      // Count seconds on the trace device.
//      trace_printf("Second %u\n", seconds);
    }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
