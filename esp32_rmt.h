extern "C" {
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
}

#define RMT_RX_ACTIVE_LEVEL  0  /*!< If we connect with a IR receiver, the data is active low */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_HEADER_HIGH_US    9050                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4510                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         150                         /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

class ESP32_RMT
{
	public:
		uint16_t result;
		ESP32_RMT(int recievePin);
    	void irRecieve();	
		
	private:
		void nec_fill_item_level(rmt_item32_t* ,int ,int);
		void nec_fill_item_header(rmt_item32_t* );
		void nec_fill_item_bit_one(rmt_item32_t* );
		void nec_fill_item_bit_zero(rmt_item32_t*);
		void nec_fill_item_end(rmt_item32_t* );
		bool nec_check_in_range(int, int, int);
		bool nec_header_if(rmt_item32_t*);
		bool nec_bit_one_if(rmt_item32_t*);
		bool nec_bit_zero_if(rmt_item32_t*);
		int nec_parse_items(rmt_item32_t*, int, uint16_t*, uint16_t*);
		int nec_build_items(int , rmt_item32_t* , int , uint16_t , uint16_t);	
		void nec_rx_init(int);
};