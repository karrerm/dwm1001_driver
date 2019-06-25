/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define APP_NAME "DS TWR INIT v1.3"

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_rx = 0;
/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1/s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 6000
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 10000
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 30

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define TX_ANT_DLY_OTP 16472
#define RX_ANT_DLY_OTP 16472	


/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on console. */
char dist_str[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

// Main Functionality
int ss_init_run(void)
{
  // Clear reception timeout to start next ranging process
  //dwt_setrxtimeout(0);

  // Activate reception immediately.
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  { };
  #if 0
  int temp = 0;		
  if(status_reg & SYS_STATUS_RXFCG )
    temp = 1;
  else if(status_reg & SYS_STATUS_RXRFTO )
    temp = 2;
  else if (status_reg & SYS_STATUS_RXPTO)
    temp = 3;
  if(status_reg & SYS_STATUS_ALL_RX_ERR )
    temp = 4;
  #endif

  if (status_reg & SYS_STATUS_RXFCG) {
    uint32 frame_len;

    // Clear good RX frame event in the DW1000 status register.
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    // A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUFFER_LEN) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    // Check that the frame is a poll sent by "DS TWR initiator" example.
    //  As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame.
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
      uint32 resp_tx_time;
      int ret;

      // Retrieve poll reception timestamp.
      poll_rx_ts = get_rx_timestamp_u64();

      // Retreive frame sequence number
      memcpy(&frame_seq_nb_rx, &rx_buffer[2], 1);

      // Set send time for response. See NOTE 9 below
      resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      // Set expected delay and timeout for final message reception. See NOTE 4 and 5 below.
      dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
      dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

      // Write and send the response message. See NOTE 10 below
      tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); // Zero offset in TX buffer.
      dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); // Zero offset in TX buffer, ranging.
      uint32 current_time;
      current_time = dwt_readsystimestamphi32();
      //printf("Current_time: %d, send_time: %d\n", current_time, resp_tx_time);
      ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

      // If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below.
      if (ret == DWT_ERROR) {
        //printf("err - tx_error1\n");
        return 0;
      }

      // Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below.
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
      { };

      #if 0  // include if required to help debug timeouts.
      int temp = 0;		
      if(status_reg & SYS_STATUS_RXFCG )
      temp = 1;
      else if(status_reg & SYS_STATUS_RXRFTO )
      temp = 2;
      else if (status_reg & SYS_STATUS_RXPTO)
      temp = 3;
      if(status_reg & SYS_STATUS_ALL_RX_ERR )
      temp = 4;
      #endif
      //printf("Status: %d\n", temp);

      // Increment frame sequence number after transmission of the response message (modulo 256). */
      frame_seq_nb++;

      if (status_reg & SYS_STATUS_RXFCG) {
        // Clear good RX frame event and TX frame sent in the DW1000 status register
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

        // A frame has been received, read it into the local buffer.
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN) {
          dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        // Check that the frame is a final message sent by "DS TWR initiator" example.
        // As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. 
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
          uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
          uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
          float clockOffsetRatio;
          double Ra, Rb, Da, Db;
          int64 tof_dtu;
          int distance_mm;

          // Retrieve response transmission and final reception timestamps.
          resp_tx_ts = get_tx_timestamp_u64();
          final_rx_ts = get_rx_timestamp_u64();

          // Get timestamps embedded in the final message.
          final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
          final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
          final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
          clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
          //printf("ClockOffsetRatio: %10.10f\r\n", clockOffsetRatio);
          // Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below.
          poll_rx_ts_32 = (uint32)poll_rx_ts;
          resp_tx_ts_32 = (uint32)resp_tx_ts;
          final_rx_ts_32 = (uint32)final_rx_ts;
          Ra = (double)(resp_rx_ts - poll_tx_ts);
          Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
          Da = (double)(final_tx_ts - resp_rx_ts);
          Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
          //Ra = (double)((resp_rx_ts - poll_tx_ts) * (1.0f - clockOffsetRatio));
          //Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
          //Da = (double)((final_tx_ts - resp_rx_ts) * (1.0f - clockOffsetRatio));
          //Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
          tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

          tof = tof_dtu * DWT_TIME_UNITS;
          
          distance = tof * SPEED_OF_LIGHT;
          distance_mm = (int)(distance * 1000.0f);

          // Display computed distance on console.
          //sprintf(dist_str, "dist (%u): %3.2f m\n", frame_seq_nb_rx, distance);
          printf("%d\n", distance_mm);
        }
      } else {
        //printf("err - rx2 failed\n");
        // Clear RX error/timeout events in the DW1000 status register.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        // Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
      }
    }
  } else {
    // printk("err 1\n"); 
    // Clear RX error/timeout events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    // Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }
}

static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dwt_setleds(DWT_LEDS_ENABLE);

  while (true){
    ss_init_run();
  }
}
/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 5. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 6. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 7. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
