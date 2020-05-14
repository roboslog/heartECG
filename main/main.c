/*------------------------------------------------------------------------------
  10/05/2020
  Author: Narendok @Roboslog
  Platforms: ESP8266
  Language: C++/Arduino
  File: ESP_Agri_thingspeak.ino
------------------------------------------
  Please consider buying products from ROBOSLOF to help fund future
  Open-Source projects like this! We'll always put our best effort in every
  project, and release all our design files and code for you to use.

  https://roboslog.in/heart
  ------------------------------------------------------------------------------
  License:
  Please see attached LICENSE.txt file for details.   
 ----------------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
//#include "esp_bt.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_master.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "esp_log.h"
#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"
#define STATS_INTERVAL 1000
#define TAG "heartypatch:"

#define PIN_NUM_0   17
#define PIN_NUM_1   14
#define PIN_NUM_2   13
#define PIN_NUM_4   2
#define PIN_NUM_5   16
#define PIN_NUM_6   26
#define PIN_NUM_7   17
#define PIN_NUM_8   25
#define PIN_NUM_10  12
#define PIN_NUM_11  23
#define PIN_NUM_12  19
#define PIN_NUM_13  18

#define PIN_NUM_A0  35
#define PIN_NUM_A1  34
#define PIN_NUM_A2  36
#define PIN_NUM_A3  39

#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22

#define PIN_SPI_CS   25
#define PIN_SPI_MOSI 23
#define PIN_SPI_MISO 19
#define PIN_SPI_SCK  18
#define BUF_SIZE  1000
#define SAMPLES_PER_PACKET 8
#define HEADER_SIZE 5
#define TRAILER_SIZE 2
#define OVERHEAD_SIZE (HEADER_SIZE+TRAILER_SIZE)
#define SAMPLE_SIZE 4
#define SEQ_SIZE 4
#define TIMESTAMP_SIZE 8
//#define RR_SIZE 8
#define RR_SIZE 4
#define PAYLOAD_SIZE (RR_SIZE + SAMPLES_PER_PACKET*SAMPLE_SIZE + SEQ_SIZE + TIMESTAMP_SIZE)
#define PAYLOAD_SIZE_LSB (PAYLOAD_SIZE &0xFF)
#define PAYLOAD_SIZE_MSB ((PAYLOAD_SIZE >> 8) &0xFF)
#define PACKET_SIZE (PAYLOAD_SIZE + HEADER_SIZE + TRAILER_SIZE)

//#define PROTOCOL_VERSION 0x02
#define PROTOCOL_VERSION 0x03
#define PACKET_SOF1 0x0A
#define PACKET_SOF2 0xFA
#define PACKET_EOF 0x0b
extern unsigned int global_heartRate ;
volatile unsigned int HR;

#define WREG 0x00
#define RREG 0x01

#define   STATUS          0x01
#define   EN_INT          0x02
#define   EN_INT2         0x03
#define   MNGR_INT        0x04
#define   MNGR_DYN        0x05
#define   SW_RST          0x08
#define   SYNCH           0x09
#define   FIFO_RST        0x0A
#define   INFO            0x0F
#define   CNFG_GEN        0x10
#define   CNFG_CAL        0x12
#define   CNFG_EMUX       0x14
#define   CNFG_ECG        0x15
#define   CNFG_RTOR1      0x1D
#define   CNFG_RTOR2      0x1E
#define   ECG_FIFO_BURST  0x20
#define   ECG_FIFO        0x21
#define   RTOR            0x25
#define   NO_OP           0x7F

#define PIN_NUM_FCLK      13

void max30003_sw_reset(void);
void max30003_start_timer(void);
void MAX30003_ReadID(void);
void MAX30003_init_sequence(void);
void max30003_initchip(int pin_miso, int pin_mosi, int pin_sck, int pin_cs );
uint8_t* max30003_read_send_data(void );


uint8_t SPI_TX_Buff[4];
uint8_t SPI_RX_Buff[10];
uint8_t DataPacketHeader[OVERHEAD_SIZE+PAYLOAD_SIZE];

signed long ecgdata;
unsigned long data;
char SPI_temp_32b[4];
spi_device_handle_t spi;

// counters
int tally_etag[8];      // histogram of etag values
int tally_reset = 0;    // ECG FIFO resets since start of current connection
int stats_read_count = 0;     // read count for reporting purposes, reset after status output


unsigned int packet_sequence_id = 0;  // packet sequence ID.  Set to 0 at start and after each FIFO reset
struct timeval timestamp;             // packet timestamp (seconds and microseconds)


void app_main()
{
        nvs_flash_init();
    max30003_initchip(PIN_SPI_MISO,PIN_SPI_MOSI,PIN_SPI_SCK,PIN_SPI_CS);
	vTaskDelay(2/ portTICK_PERIOD_MS);
 ESP_LOGI(TAG, "start");
  //  max30003_reg_read();
 // max30003_spi_pre_transfer_callback();
 while (1)
 {
    MAX30003_ReadID();
    max30003_read_send_data();        //* code */
 }
 
   //uart_write_bytes(uart_num, (const char *) uart_data, 25);
        //uart_tx_chars(uart_num, (const char *) uart_data, 5);
        //putc('a',stdout);
 }
//This function is called (in irq context!) just before a transmission starts.
void max30003_spi_pre_transfer_callback(spi_transaction_t *t)
{
;
}

void max30003_start_timer(void)
{
    ledc_timer_config_t ledc_timer =
    {
    .duty_resolution = LEDC_TIMER_13_BIT, //set timer counter bit number
      .freq_hz = 4096 ,             //set frequency of pwm 32768
      .speed_mode = LEDC_HIGH_SPEED_MODE,   //timer mode,
      .timer_num = 0    //timer index
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel =
    {
        .channel = LEDC_CHANNEL_0,
        .duty = 512,
        .gpio_num = PIN_NUM_FCLK,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };

    ledc_channel_config(&ledc_channel);
}


void MAX30003_Reg_Write (unsigned char WRITE_ADDRESS, unsigned long data)
{
  uint8_t wRegName = (WRITE_ADDRESS<<1) | WREG;

  uint8_t txData[4];

  txData[0]=wRegName;
  txData[1]=(data>>16);
  txData[2]=(data>>8);
  txData[3]=(data);

  esp_err_t ret;
  spi_transaction_t t;

  memset(&t, 0, sizeof(t));       //Zero out the transaction

  t.length=32;                 //Len is in bytes, transaction length is in bits.
  t.tx_buffer=&txData;               //Data
  ret=spi_device_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
}


void MAX30003_ReadID(void)
{
   uint8_t SPI_TX_Buff[4];
   uint8_t SPI_RX_Buff[10];

   uint8_t Reg_address=INFO;

   SPI_TX_Buff[0] = (Reg_address<<1 ) | RREG;
   SPI_TX_Buff[1]=0x00;
   SPI_TX_Buff[2]=0x00;
   SPI_TX_Buff[3]=0x00;

   esp_err_t ret;
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));       //Zero out the transaction

   t.length=32;                     //Command is 8 bits
   t.rxlength=32;
   t.tx_buffer=&SPI_TX_Buff;               //The data is the cmd itself
   t.rx_buffer=&SPI_RX_Buff;

   t.user=(void*)0;                //D/C needs to be set to 0
   ret=spi_device_transmit(spi, &t);  //Transmit!
   assert(ret==ESP_OK);            //Should have had no issues.
}


void max30003_reg_read(unsigned char WRITE_ADDRESS)
{
    uint8_t Reg_address=WRITE_ADDRESS;

    SPI_TX_Buff[0] = (Reg_address<<1 ) | RREG;
    SPI_TX_Buff[1]=0x00;
    SPI_TX_Buff[2]=0x00;
    SPI_TX_Buff[3]=0x00;

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=32;
    t.rxlength=32;
    t.tx_buffer=&SPI_TX_Buff;
    t.rx_buffer=&SPI_RX_Buff;

    t.user=(void*)0;
    ret=spi_device_transmit(spi, &t);
    assert(ret==ESP_OK);            //Should have had no issues.
ESP_LOGI(TAG, "\n Spi tx");
    SPI_temp_32b[0] = SPI_RX_Buff[1];
    SPI_temp_32b[1] = SPI_RX_Buff[2];
    SPI_temp_32b[2] = SPI_RX_Buff[3];

}


void max30003_sw_reset(void)
{
    MAX30003_Reg_Write(SW_RST,0x000000);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}


void max30003_synch(void)
{
    MAX30003_Reg_Write(SYNCH,0x000000);
}


void max30003_fifo_reset(void)
{
    MAX30003_Reg_Write(FIFO_RST,0x000000);
    tally_reset++;
}


void MAX30003_init_sequence() {
    int i;
    max30003_fifo_reset();
    ESP_LOGI(TAG, "fifo Resetting\n");
    for (i=0; i < 8; i++)
        tally_etag[i] = 0;

    tally_reset = 0;
    stats_read_count = 0;
    packet_sequence_id = 0;
}


void print_counters() {
    int i;
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "Tally for FIFO Reset: %d", tally_reset);
    ESP_LOGI(TAG, "Tally for ETag");
    for (i=0; i < 8; i++)
        ESP_LOGI(TAG, "ETag: %x count %d", i, tally_etag[i]);
    ESP_LOGI(TAG, "\n");
}


void max30003_initchip(int pin_miso, int pin_mosi, int pin_sck, int pin_cs )
{
    esp_err_t ret;

    spi_bus_config_t buscfg=
    {
        .miso_io_num=pin_miso,
        .mosi_io_num=pin_mosi,
        .sclk_io_num=pin_sck,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    spi_device_interface_config_t devcfg=
    {
        .clock_speed_hz=4000000,             	  //Clock out at 10 MHz
        .mode=0,                               	 //SPI mode 0
        .spics_io_num=pin_cs,              		 //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=max30003_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 0);				//use 1 instead of 0 to enable dma
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    max30003_start_timer();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    max30003_sw_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    MAX30003_Reg_Write(CNFG_GEN, 0x080004);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    MAX30003_Reg_Write(CNFG_CAL, 0x720000);  // 0x700000
    vTaskDelay(100 / portTICK_PERIOD_MS);

    MAX30003_Reg_Write(CNFG_EMUX,0x0B0000);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    unsigned long ecg_config = 0x001000;    // was 0x005000
#ifdef CONFIG_SPS_128
    ecg_config = ecg_config | 0x800000;   //  d[23:22] -- RATE[0:1]: 10 for 128sps
        ESP_LOGI(TAG, "max30003_initchip setting SPS to 128");
#endif
#ifdef CONFIG_SPS_256
    ecg_config = ecg_config | 0x400000;   //  d[23:22] -- RATE[0:1]: 01 for 256 sps
    ESP_LOGI(TAG, "max30003_initchip setting SPS to 256");
#endif
#ifdef CONFIG_SPS_512
    //ecg_config = ecg_config | 0x000000;   //  d[23:22] -- RATE[0:1]: 00 for 512sps
    ESP_LOGI(TAG, "max30003_initchip setting SPS to 512");
#endif

#ifdef CONFIG_DHPF_ENABLE
    ecg_config = ecg_config | 0x004000;   //  d[14] = enable 0.5Hz filter
    ESP_LOGI(TAG, "max30003_initchip DHPF Enabled");
#else
    ESP_LOGI(TAG, "max30003_initchip DHPF Disabled");
#endif


//ecg_config = 0x001000;    // TODO: Delete me -- for debug purposes only
    MAX30003_Reg_Write(CNFG_ECG, ecg_config);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    MAX30003_Reg_Write(CNFG_RTOR1,0x3fc600);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    unsigned mngr_int = 0x4 | (SAMPLES_PER_PACKET - 1) << 19;
    MAX30003_Reg_Write(MNGR_INT, mngr_int);
    vTaskDelay(100 / portTICK_PERIOD_MS);

	MAX30003_Reg_Write(EN_INT,0x000400);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    max30003_synch();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    MAX30003_init_sequence();

    
}


int max30003_read_ecg_data(int ptr)
{
      max30003_reg_read(ECG_FIFO);

      unsigned long data0 = (unsigned long) (SPI_temp_32b[0]);
      data0 = data0 <<24;
      unsigned long data1 = (unsigned long) (SPI_temp_32b[1]);
      data1 = data1 <<16;
      unsigned long data2 = (unsigned long) (SPI_temp_32b[2]);
      data2 = data2 & 0xc0;
      data2 = data2 << 8;
      data = (unsigned long) (data0 | data1 | data2);
      ecgdata = (signed long) (data);

      DataPacketHeader[ptr] = ecgdata;
      DataPacketHeader[ptr+1] = ecgdata>>8;
      DataPacketHeader[ptr+2] = ecgdata>>16;
      DataPacketHeader[ptr+3] = ecgdata>>24;

      unsigned char ecg_etag = (SPI_temp_32b[2] >> 3) & 0x7;
      tally_etag[ecg_etag]++;
      stats_read_count++;

      return SAMPLE_SIZE;
}


int max30003_read_rtor_data(int ptr)
{
    max30003_reg_read(RTOR);
    unsigned long RTOR_msb = (unsigned long) (SPI_temp_32b[0]);
    unsigned char RTOR_lsb = (unsigned char) (SPI_temp_32b[1]);
    unsigned long rtor = (RTOR_msb<<8 | RTOR_lsb);
    rtor = ((rtor >>2) & 0x3fff) ;
    unsigned int RR = (unsigned int)rtor*8 ;  //8ms

    DataPacketHeader[ptr] = RR;
    DataPacketHeader[ptr+1] = RR>>8;
    DataPacketHeader[ptr+2] = 0x00;
    DataPacketHeader[ptr+3] = 0x00;

    
    float hr =  60 /((float)rtor*0.008);
    unsigned int HR = (unsigned int)hr;  // type cast to int
    ESP_LOGI(TAG, "The heart Rate : %d",HR);
    DataPacketHeader[ptr+4] = HR;
    DataPacketHeader[ptr+5] = HR>>8;
    DataPacketHeader[ptr+6] = 0x00;
    DataPacketHeader[ptr+7] = 0x00;
    

    return RR_SIZE;
}


int max30003_include_packet_sequence_id(int ptr)
{
    DataPacketHeader[ptr] = packet_sequence_id;
    DataPacketHeader[ptr+1] = packet_sequence_id>>8;
    DataPacketHeader[ptr+2] = packet_sequence_id>>16;
    DataPacketHeader[ptr+3] = packet_sequence_id>>24;
    packet_sequence_id++;

    return SEQ_SIZE;
}


int max30003_include_timestamp(int ptr)
{
    gettimeofday(&timestamp, NULL);

    DataPacketHeader[ptr] = timestamp.tv_sec;
    DataPacketHeader[ptr+1] = timestamp.tv_sec>>8;
    DataPacketHeader[ptr+2] = timestamp.tv_sec>>16;
    DataPacketHeader[ptr+3] = timestamp.tv_sec>>24;

    DataPacketHeader[ptr+4] = timestamp.tv_usec;
    DataPacketHeader[ptr+5] = timestamp.tv_usec>>8;
    DataPacketHeader[ptr+6] = timestamp.tv_usec>>16;
    DataPacketHeader[ptr+7] = timestamp.tv_usec>>24;

    return TIMESTAMP_SIZE ;
}






uint8_t* max30003_read_send_data(void)
{
    int size;
    int ptr;

    max30003_reg_read(STATUS);
    printf("\nThe status bits ......");


    unsigned long RTOR_msb = (unsigned long) (SPI_temp_32b[0]);
    unsigned char RTOR_lsb = (unsigned char) (SPI_temp_32b[1]);

    unsigned long rtor = (RTOR_msb<<8 | RTOR_lsb);
    rtor = ((rtor >>2) & 0x3fff) ;

    float hr =  60 /((float)rtor*0.0078125);
    
    //float rtor_ms = ((float)rtor*0.0078125);
	  
 //  global_heartRate= (unsigned int)hr;

   HR = (unsigned int)hr;  // type cast to int

   printf("\nThe hr rate %u",HR);
   // HR = (unsigned int)hr;  // type cast to int
   // RR = (unsigned int)((float)rtor*7.8125) ;  //8ms
    uint8_t status_bits = (SPI_temp_32b[0] >> 4) & 0xF;
   printf("\nThe status bit is %u",status_bits);

    if ((status_bits & 0x4) == 0x4) {
        // Reset EOVF condition
        ESP_LOGI(TAG, "FIFO Reset");
        max30003_fifo_reset();
        packet_sequence_id = 0;
        return NULL;
    }

    if ((status_bits & 0x8) == 0) {
        // No data present in FIFO
        return NULL;
    }

#if CONFIG_MAX30003_STATS_ENABLE
    if (stats_read_count > STATS_INTERVAL) {
        print_counters();
        stats_read_count = 0;
    }
#endif
    ESP_LOGI(TAG, "CONFIG_MAX30003_STATS -Data pcka hader");

    DataPacketHeader[0] = PACKET_SOF1;
    DataPacketHeader[1] = PACKET_SOF2;
    DataPacketHeader[2] = PAYLOAD_SIZE_LSB;
    DataPacketHeader[3] = PAYLOAD_SIZE_MSB;
    DataPacketHeader[4] = PROTOCOL_VERSION;
    ptr = HEADER_SIZE;
  printf("The header size is %d",ptr);

    size = max30003_include_packet_sequence_id(ptr);
    ptr += size;

    size = max30003_include_timestamp(ptr);
    ptr += size;


    // Fetch RR interval data
    size = max30003_read_rtor_data(ptr);
    ptr += size;
  printf("The ROR is %d",ptr);


    // Fetch ECG data
    int i;
    for (i=0; i <SAMPLES_PER_PACKET; i++) {
        size = max30003_read_ecg_data(ptr);
        ptr += size;
    }
  printf("The header is %d",ptr);

    DataPacketHeader[ptr] = 0xF0;   // 0x00;
    DataPacketHeader[ptr+1] = PACKET_EOF;

	return DataPacketHeader;
  }



 
