
#ifdef __cplusplus
 extern "C" {
#endif
/**
  ******************************************************************************
  * @file           : app_x-cube-ai.c
  * @brief          : AI program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
 /*
  * Description
  *   v1.0 - Minimum template to show how to use the Embedded Client API
  *          model. Only one input and one output is supported. All
  *          memory resources are allocated statically (AI_NETWORK_XX, defines
  *          are used).
  *          Re-target of the printf function is out-of-scope.
  *
  *   For more information, see the embeded documentation:
  *
  *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
  *
  *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
  *   typical : C:\Users\<user_name>\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\6.0.0
  */
/* Includes ------------------------------------------------------------------*/
/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"
#include "ai_datatypes_defines.h"

/* USER CODE BEGIN includes */
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_lcd.h"
#include <stdbool.h>
#include "dwt_utility.h"
#include "img_preprocess.h"

#define PATCH_SIZE 28

#define PEN_POINT_SIZE 7
#define TOUCH_TIMEOUT 700

#define PATCH_SIZE_X 58
#define PATCH_SIZE_Y 32
int complete = 0;

static uint32_t g_patch8888_5832[PATCH_SIZE_X * PATCH_SIZE_Y];
static uint32_t g_patch8888_2828[28 * 28];

static ai_float in_data[AI_NETWORK_IN_1_SIZE];
static ai_float out_data[AI_NETWORK_OUT_1_SIZE] = { 0 };
/* USER CODE END includes */
/* Global AI objects */
static ai_handle network = AI_HANDLE_NULL;
static ai_network_report network_info;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(4)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/*  In the case where "--allocate-inputs" option is used, memory buffer can be
 *  used from the activations buffer. This is not mandatory.
 */
#if !defined(AI_NETWORK_INPUTS_IN_ACTIVATIONS)
/* Allocate data payload for input tensor */
AI_ALIGNED(4)
static ai_u8 in_data_s[AI_NETWORK_IN_1_SIZE_BYTES];
#endif

/*  In the case where "--allocate-outputs" option is used, memory buffer can be
 *  used from the activations buffer. This is no mandatory.
 */
#if !defined(AI_NETWORK_OUTPUTS_IN_ACTIVATIONS)
/* Allocate data payload for the output tensor */
AI_ALIGNED(4)
static ai_u8 out_data_s[AI_NETWORK_OUT_1_SIZE_BYTES];
#endif

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
	if (fct)
		printf("TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
				err.type, err.code);
	else
		printf("TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type,
				err.code);

	do {
	} while (1);
  /* USER CODE END log */
}

static int ai_boostrap(ai_handle w_addr, ai_handle act_addr)
{
  ai_error err;

  /* 1 - Create an instance of the model */
  err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    ai_log_err(err, "ai_network_create");
    return -1;
  }

  /* 2 - Initialize the instance */
  const ai_network_params params = {
      AI_NETWORK_DATA_WEIGHTS(w_addr),
      AI_NETWORK_DATA_ACTIVATIONS(act_addr) };

  if (!ai_network_init(network, &params)) {
      err = ai_network_get_error(network);
      ai_log_err(err, "ai_network_init");
      return -1;
    }

  /* 3 - Retrieve the network info of the created instance */
  if (!ai_network_get_info(network, &network_info)) {
    err = ai_network_get_error(network);
    ai_log_err(err, "ai_network_get_error");
    ai_network_destroy(network);
    network = AI_HANDLE_NULL;
    return -3;
  }

  return 0;
}

static int ai_run(void *data_in, void *data_out)
{
  ai_i32 batch;

  ai_buffer *ai_input = network_info.inputs;
  ai_buffer *ai_output = network_info.outputs;

  ai_input[0].data = AI_HANDLE_PTR(data_in);
  ai_output[0].data = AI_HANDLE_PTR(data_out);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    ai_log_err(ai_network_get_error(network),
        "ai_network_run");
    return -1;
  }

  return 0;
}

/* USER CODE BEGIN 2 */
char password[7] = { 'W', 'A', 'K', 'E', 'U', 'P' };
int stage = 0;
int acquire_and_process_data(void *data) {
	static TS_StateTypeDef ts_state;
	static uint32_t touch_time;
	static bool data_ready = false;
	static uint32_t t_cycle_start, t_cycle_inference_duration;
	static struct dwtTime t_ms_inference_duration;
	static float max_probability;
	static int8_t max_index;
	static char prediction;
	static char msg[30];


	if (stage < 7) {

		/* Get status and positions of the touch screen */
		if (BSP_TS_GetState(&ts_state) == TS_OK) {
			/* Touch event*/
			if (ts_state.touchDetected > 0 && data_ready == false) {
				BSP_LCD_Clear(LCD_COLOR_BLACK);
				data_ready = true;
			}

			/*Draw display*/
			for (int i = 0; i < ts_state.touchDetected; i++) {
				touch_time = HAL_GetTick();
				BSP_LCD_FillCircle(ts_state.touchX[i], ts_state.touchY[i],
				PEN_POINT_SIZE);
			}

			/* Major block of drawing mini patch and running AI related code */
			if (data_ready && (HAL_GetTick() - touch_time) > TOUCH_TIMEOUT) {
				/* retrieve downsampled image 58*32 from display memory (480*272) in SDRAM */
				int ii = 0;

				for (uint16_t y = 16; y < BSP_LCD_GetYSize(); y += 8) {
					for (uint16_t x = 16; x < BSP_LCD_GetXSize(); x += 8) {
						g_patch8888_5832[ii++] = BSP_LCD_ReadPixel(x, y);
					}
				}

				/* Resize from 58*32 to 28*28 */
				ImageResize((uint8_t*) g_patch8888_5832, 58, 32, 4, 0, 0, 58,
						32, (uint8_t*) g_patch8888_2828, 28, 28);

				BSP_LCD_Clear(LCD_COLOR_BLACK);

				/* Draw 28*28 and prepare input array of AI model: in_data */
				ii = 0;
				for (uint16_t y = 0; y < PATCH_SIZE; y += 1) {
					for (uint16_t x = 0; x < PATCH_SIZE; x += 1) {
						in_data[ii] =
								(g_patch8888_2828[ii] > 0xFF000000) ?
										1.0F : 0.0F;
						BSP_LCD_DrawPixel(x, y, g_patch8888_2828[ii++]);
					}
				}

				/* DWT reset*/
				dwtReset();

				/* inference duration measurement */
				t_cycle_start = dwtGetCycles();

				/* run NN*/
				ai_run(in_data, out_data);

				/* inference duration measurement */
				t_cycle_inference_duration = dwtGetCycles() - t_cycle_start;

				dwtCyclesToTime(t_cycle_inference_duration,
						&t_ms_inference_duration);

				/* show results*/
				max_probability = 0;
				max_index = -1;
				for (ii = 0; ii < AI_NETWORK_OUT_1_SIZE; ii++) {
					if (out_data[ii] > max_probability) {
						max_probability = out_data[ii];
						max_index = ii;
					}
				}

				BSP_LCD_SetFont(&Font24);
				if (max_index >= 0 && max_probability > 0.7F) {
					prediction =
							(max_index < 10) ? max_index + 48 : max_index + 55;
					sprintf(msg, "= %c", prediction);
					BSP_LCD_SetFont(&Font16);
					BSP_LCD_DisplayStringAt(38, 6, (uint8_t*) msg, LEFT_MODE);
					sprintf(msg, "(%d%%)", (int) (max_probability * 100));
					BSP_LCD_DisplayStringAt(90, 4, (uint8_t*) msg, LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_RED);
					BSP_LCD_SetFont(&Font24);
					if (prediction == password[stage]) {
						BSP_LCD_DisplayStringAt(80, 35, (uint8_t*) "Bingo!",
								CENTER_MODE);
						stage += 1;
					} else {
						BSP_LCD_DisplayStringAt(80, 35,
								(uint8_t*) "Not this one!", CENTER_MODE);
					}
				} else {
					BSP_LCD_DisplayStringAt(30, 35,
							(uint8_t*) " can't recognize...", RIGHT_MODE);
				}
				int wpos = 280;
				int npos = wpos + 11 * stage;
				BSP_LCD_SetFont(&Font16);
				switch (stage) {
				case 0:
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(wpos, 4, (uint8_t*) "WAKEUP",
							LEFT_MODE);
					break;
				case 1:
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(wpos, 4, (uint8_t*) "W",
							LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(npos, 4, (uint8_t*) "AKEUP",
							LEFT_MODE);
					break;
				case 2:
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(wpos, 4, (uint8_t*) "WA",
							LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(npos, 4, (uint8_t*) "KEUP",
							LEFT_MODE);
					break;
				case 3:
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(wpos, 4, (uint8_t*) "WAK",
							LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(npos, 4, (uint8_t*) "EUP",
							LEFT_MODE);
					break;
				case 4:
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(wpos, 4, (uint8_t*) "WAKE",
							LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(npos, 4, (uint8_t*) "UP",
							LEFT_MODE);
					break;
				case 5:
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(wpos, 4, (uint8_t*) "WAKEU",
							LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(npos, 4, (uint8_t*) "P",
							LEFT_MODE);
					break;
				case 6:
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(wpos, 4, (uint8_t*) "WAKEUP",
							LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
					BSP_LCD_DisplayStringAt(50, 85,
							(uint8_t*) "You're awake now!", CENTER_MODE);
					HAL_Delay(3000);
					BSP_LCD_Clear(LCD_COLOR_BLACK);
					stage++;
					break;
				}

				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				data_ready = false;
			} // end of if( data_ready && (HAL_GetTick() - touch_time) > TOUCH_TIMEOUT)
		} // end of if( BSP_TS_GetState(&ts_state) == TS_OK )
		return 0;
	} else {
		complete=1;
		/* Get status and positions of the touch screen */
		if (BSP_TS_GetState(&ts_state) == TS_OK) {
			/* Touch event*/

			if (ts_state.touchDetected > 0 && data_ready == false) {
				BSP_LCD_Clear(LCD_COLOR_BLACK);
				data_ready = true;
			}

			/*Draw display*/
			for (int i = 0; i < ts_state.touchDetected; i++) {
				touch_time = HAL_GetTick();
				BSP_LCD_FillCircle(ts_state.touchX[i], ts_state.touchY[i],
				PEN_POINT_SIZE);
			}

			/* Major block of drawing mini patch and running AI related code */
			if (data_ready && (HAL_GetTick() - touch_time) > TOUCH_TIMEOUT) {
				/* retrieve downsampled image 58*32 from display memory (480*272) in SDRAM */
				int ii = 0;

				for (uint16_t y = 16; y < BSP_LCD_GetYSize(); y += 8) {
					for (uint16_t x = 16; x < BSP_LCD_GetXSize(); x += 8) {
						g_patch8888_5832[ii++] = BSP_LCD_ReadPixel(x, y);
					}
				}

				/* Resize from 58*32 to 28*28 */
				ImageResize((uint8_t*) g_patch8888_5832, 58, 32, 4, 0, 0, 58,
						32, (uint8_t*) g_patch8888_2828, 28, 28);

				BSP_LCD_Clear(LCD_COLOR_BLACK);

				/* Draw 28*28 and prepare input array of AI model: in_data */
				ii = 0;
				for (uint16_t y = 0; y < PATCH_SIZE; y += 1) {
					for (uint16_t x = 0; x < PATCH_SIZE; x += 1) {
						in_data[ii] =
								(g_patch8888_2828[ii] > 0xFF000000) ?
										1.0F : 0.0F;
						BSP_LCD_DrawPixel(x, y, g_patch8888_2828[ii++]);
					}
				}

				/* DWT reset*/
				dwtReset();

				/* inference duration measurement */
				t_cycle_start = dwtGetCycles();

				/* run NN*/
				ai_run(in_data, out_data);

				/* inference duration measurement */
				t_cycle_inference_duration = dwtGetCycles() - t_cycle_start;

				dwtCyclesToTime(t_cycle_inference_duration,
						&t_ms_inference_duration);

				/* show results*/
				max_probability = 0;
				max_index = -1;
				for (ii = 0; ii < AI_NETWORK_OUT_1_SIZE; ii++) {
					if (out_data[ii] > max_probability) {
						max_probability = out_data[ii];
						max_index = ii;
					}
				}

				BSP_LCD_SetFont(&Font24);
				if (max_index >= 0 && max_probability > 0.7F) {
					prediction =
							(max_index < 10) ? max_index + 48 : max_index + 55;
					sprintf(msg, "= %c", prediction);
					BSP_LCD_SetFont(&Font16);
					BSP_LCD_DisplayStringAt(38, 6, (uint8_t*) msg, LEFT_MODE);
					sprintf(msg, "(%d%%)", (int) (max_probability * 100));
					BSP_LCD_DisplayStringAt(90, 4, (uint8_t*) msg, LEFT_MODE);
					BSP_LCD_SetTextColor(LCD_COLOR_RED);
					BSP_LCD_SetFont(&Font24);
				} else {
					BSP_LCD_DisplayStringAt(30, 35,
							(uint8_t*) " can't recognize...", RIGHT_MODE);
				}
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				data_ready = false;
			} // end of if( data_ready && (HAL_GetTick() - touch_time) > TOUCH_TIMEOUT)
		} // end of if( BSP_TS_GetState(&ts_state) == TS_OK )
		return 0;
	}

}

int post_process(void *data) {
	return 0;
}
/* USER CODE END 2 */

/*************************************************************************
  *
  */
void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
	printf("\r\nTEMPLATE - initialization\r\n");

	ai_boostrap(ai_network_data_weights_get(), activations);
    /* USER CODE END 5 */
}

int MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */

	int res = -1;
	uint8_t *in_data = NULL;
	uint8_t *out_data = NULL;

	printf("TEMPLATE - run - main loop\r\n");

	if (network) {

		if ((network_info.n_inputs != 1) || (network_info.n_outputs != 1)) {
			ai_error err =
					{ AI_ERROR_INVALID_PARAM, AI_ERROR_CODE_OUT_OF_RANGE };
			ai_log_err(err,
					"template code should be updated\r\n to support a model with multiple IO");
			return;
		}

		/* 1 - Set the I/O data buffer */

#if AI_NETWORK_INPUTS_IN_ACTIVATIONS
    in_data = network_info.inputs[0].data;
#else
		in_data = in_data_s;
#endif

#if AI_NETWORK_OUTPUTS_IN_ACTIVATIONS
    out_data = network_info.outputs[0].data;
#else
		out_data = out_data_s;
#endif

		if ((!in_data) || (!out_data)) {
			printf("TEMPLATE - I/O buffers are invalid\r\n");
			return;
		}

		/* 2 - main loop */
		do {
			/* 1 - acquire and pre-process input data */
			res = acquire_and_process_data(in_data);
			/* 2 - process the data - call inference engine */
			if (res == 0)
				res = ai_run(in_data, out_data);
			/* 3- post-process the predictions */
			if (res == 0)
				res = post_process(out_data);
			if (complete==1)
				return 1;

		} while (res == 0);
	}

	if (res) {
		ai_error err = { AI_ERROR_INVALID_STATE, AI_ERROR_CODE_NETWORK };
		ai_log_err(err, "Process has FAILED");
	}
    /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
