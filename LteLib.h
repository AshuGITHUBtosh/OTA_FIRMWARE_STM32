/*
 * ltelib.h
 *
 *  Created on: 06-Jul-2021
 *      Author: harsh
 */

#ifndef INC_LTELIB_H_
#define INC_LTELIB_H_


#include <stdarg.h>
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"

#include "def.h"

#define NetworkHandle huart3

extern void reset_rx_buffer ( void );

char netw_serv_provider[4];
//char MQTTbroker[64];
extern uint8_t netw_rx_flag;
extern uint8_t uart_input_buffer[UART_INPUT_BUF_SIZE];
extern void reset_rx_buffer ( void );

char netw_info_buffer[64];
char ip_adr[16];
extern int8_t rssi_current;
uint8_t msg_packet[128];

extern char dash_status[40];
extern char dash_response[32];
extern char gps_adr[32];
extern uint8_t gps_start_count;


//	Gv FOR TIME & DATE
extern uint8_t date, month, year;
extern uint8_t hour, min, sec;
extern float timezone;

extern float Battery_SOC;
extern float CPU_temperature;
extern float run_len;
extern float total_run_len;

// ----------------------

uint8_t timestampLocal[24] ;


char *lte_strstr(char *str_haystack, char *str_pin, uint16_t size_haystack )
{
	uint16_t temp_count = 0;
	while(*(str_haystack+temp_count) == '\0' && temp_count < size_haystack)
	{
		temp_count++;
	}

	if(temp_count == size_haystack)
	{
		return NULL;
	}
	else
	{
		return strstr((str_haystack+temp_count), str_pin);
	}
}


uint8_t	lte_command_response (bool empty_buffer, uint8_t *at_command ,
							  uint32_t max_waiting_ms ,
							  uint8_t no_of_response, ...)
{
	// Extract Possible Responses
	va_list tag;
	va_start( tag , no_of_response ) ;
	char *arg[no_of_response];

	for (uint8_t tmp_i = 0; tmp_i < no_of_response; tmp_i++ )
	{
		arg[tmp_i] = va_arg( tag, char * );
	}

	va_end ( tag );

	//TEST
	reset_rx_buffer();
	HAL_UART_Receive_DMA(&NetworkHandle, uart_input_buffer, UART_INPUT_BUF_SIZE);

	HAL_UART_Transmit ( &NetworkHandle , (uint8_t *) at_command , strlen((char*)at_command) , HAL_MAX_DELAY );
	//HAL_UART_Transmit ( &huart4 , (uint8_t *) ".>." , 3 , HAL_MAX_DELAY );

	//HAL_UART_Transmit ( &huart4 , (uint8_t *) at_command , strlen((char*)at_command) , HAL_MAX_DELAY );
	HAL_UART_Transmit ( &NetworkHandle , (uint8_t *) "\r" , 1 , HAL_MAX_DELAY );



	//Check received Response
	while(max_waiting_ms)
	{
		osDelay(10);

		//if( 1 == lte_rx_flag )
		{

			for( uint8_t tmp_j = 0; tmp_j < no_of_response; tmp_j++)
			{

				if(lte_strstr( (char*) uart_input_buffer, arg[tmp_j], sizeof(uart_input_buffer)) != 0)
	//			if(strstr( (char*) uart_input_buffer + 1,arg[tmp_j]) != 0)
				{

					if(1 == empty_buffer)
					{
						reset_rx_buffer();
					}

					return ++(tmp_j);
				}

			}

		}

		max_waiting_ms-=10;

	}

//	reset_rx_buffer();
	return 0;
}


uint8_t lte_hard_reset(void)
{
	// LTE_PERST# to be pulled LOW for atleast 300ms to Hard Reset
	HAL_GPIO_WritePin(LTE_PERST__GPIO_Port, LTE_PERST__Pin, GPIO_PIN_RESET);
	osDelay(400);
	HAL_GPIO_WritePin(LTE_PERST__GPIO_Port, LTE_PERST__Pin, GPIO_PIN_SET);

	DEBUG_PRINT(("Waiting for Reset Response!\r\n"));

	// Check for RDY to Confirm Successfull reset
	int8_t attempts = 50;
	while(attempts--)
	{
		if(strstr( (char*) uart_input_buffer,"RDY") != 0)
			break;

		if(attempts){
			osDelay(200);
		}
		else{
			return 0;
		}
	}
	osDelay(100);

	attempts = 10;
	while(attempts--)
	{
		if(strstr( (char*) uart_input_buffer,"+QIND: PB DONE") != 0)
			break;

		if(attempts){
			osDelay(1000);
			DEBUG_PRINT((".\r\n"));
		}
		else{
			return 0;
		}
	}

	reset_rx_buffer();
	return 1;
}


uint8_t lte_atok(void)				// add "uint8_t lte_at(void);" in header
{
	uint8_t rx_response;
	rx_response = lte_command_response(1,  (uint8_t *)"AT", 10000, 1, "OK");

	if(1 == rx_response)
	{
		#if WDT
			HAL_IWDG_Refresh(&hiwdg);
		#endif

		return 1;
	}
	return 0;
}

uint8_t lte_echo_mode_off(void)
{
	uint8_t rx_response;
	rx_response = lte_command_response(1,  (uint8_t *)"ATE0", 500, 1, "OK");

	if(1 == rx_response)
	{
		printf("Echo mode OFF\r\n");
		return 1;
	}

	printf("Unable to reset echo mode\r\n");
	return 0;
}


//	Service Provider APN Settings
uint8_t lte_set_service_provider(void)
{
	uint8_t rx_response;
	rx_response = lte_command_response(1,(uint8_t *) "AT+COPS?", 2000, 7,
			"IND airtel", "Airtel", "airtel", "JIO", "IND-JIO", "Vi India", "ERROR");


	if( (1 == rx_response)||(2 == rx_response)||(3 == rx_response) )
	{
		rx_response = 0;
		rx_response = lte_command_response(1,(uint8_t *) "AT+QICSGP=1,1,\"airtelgprs.com\"",
								   2000, 2, "OK", "ERROR" );
		sprintf(netw_serv_provider, "AIR");
		return 1;
	}
	else if( (4 == rx_response)||(5 == rx_response))
	{
		rx_response = 0;
		rx_response = lte_command_response(1,(uint8_t *) "AT+QICSGP=1,2,\"jionet\"", 4000, 2, "OK", "ERROR" );
		sprintf(netw_serv_provider, "JIO");
		return 1;
	}
	else if( 6 == rx_response)
	{
		rx_response = 0;
		rx_response = lte_command_response(1,(uint8_t *) "AT+QICSGP=1,1,\"VI Net Speed\"",
								   2000, 2, "OK", "ERROR" );
		sprintf(netw_serv_provider, "VI.");
		return 1;
	}
	return 0;

}


//	Connect to GPRS
uint8_t lte_activate_pdp(void)
{
	uint8_t rx_response;
	rx_response = lte_command_response(1,(uint8_t *) "AT+QIACT=1", 30000, 2,
									   "OK", "ERROR" );
	if( 1 == rx_response )
	{
		//printf("GSM | GPS\t: Connected to GPRS\r\n");
		#if WDT
			  HAL_IWDG_Refresh(&hiwdg);
		#endif

		return 1;
	}
	return 0;
}

// configurations for MQTT
uint8_t lte_mqtt_conf(void)
{
	if( lte_command_response(1,(uint8_t *) "AT+QMTCFG=\"version\",0,4", 3000, 2,
			   "OK", "ERROR") )
	{
		if( lte_command_response(1,(uint8_t *) "AT+QMTCFG=\"pdpcid\",0,1", 1000, 2,
				   "OK", "ERROR") )
			return 1;
	}
	return 0;
}

uint8_t lte_mqtt_open(uint8_t client_id, char* server_ip, uint16_t port)
{

	char temp_cmd_buffer[64];
	sprintf(temp_cmd_buffer, "AT+QMTOPEN=%d,\"%s\",%d",
			client_id, server_ip, port);

	if( lte_command_response(1,(uint8_t *) temp_cmd_buffer, 10000, 2,
			   "+QMTOPEN: 1,0", "ERROR") )
		return 1;

	return 0;
}


uint8_t lte_mqtt_conn(uint8_t client_id, char* bot_i,
						char* username, char* password )
{
	char temp_cmd_buffer[64];
	sprintf(temp_cmd_buffer, "AT+QMTCONN=%d,\"%s\",\"%s\",\"%s\"",
			client_id, bot_id, username, password);

	if( lte_command_response(1,(uint8_t *) temp_cmd_buffer, 20000, 2, "OK", "ERROR" ) )
		return 1;

	return 0;
}

uint8_t mqtt_subscribe (uint8_t client_id, char* topic, uint8_t qos)
{
	char temp_cmd_buffer[64];

	sprintf(temp_cmd_buffer, "AT+QMTSUB=%d,1,\"%s/%s\",%d",
			(int) client_id, bot_id, topic, qos);

	if( lte_command_response(1,(uint8_t *) temp_cmd_buffer,
			   20000, 2, "+QMTSUB: 1,1,0", "ERROR") )
		return 1;

	return 0;
}

uint8_t mqtt_publish (uint8_t client_id, char* topic, char* message, uint8_t qos)
{

	//uint16_t mesg_id = ++mesg_id_pool;
	char publish_cmd_mesg_buffer[64];		// Contains either command or message at a time
	uint16_t mesg_len = strlen((char *)bot_id) + 1 + strlen(message); // <bot_id>;<message>

	//Command Formation
	sprintf(publish_cmd_mesg_buffer,"AT+QMTPUBEX=%d,0,%d,0,\"%s\",%d",
									client_id, qos, topic, mesg_len);

	//printf("%s\r\n",publish_cmd_mesg_buffer);

	//Send Publish Command & wait for '>'
	//lte_command_response(1,(uint8_t *) publish_cmd_mesg_buffer,
									   //100, 2, ">", "ERROR" );
	uint8_t rx_response;
	rx_response = lte_command_response(1,(uint8_t *) publish_cmd_mesg_buffer,
			   	   	   	   	   	   	   	100, 2, ">", "ERROR" );

	if (1 != rx_response)
		return 0;
	//printf("RX_res-1: %d\r\n",rx_response);

	memset((char*) publish_cmd_mesg_buffer, 0, sizeof(publish_cmd_mesg_buffer));
	sprintf(publish_cmd_mesg_buffer, "%s;%s", bot_id, message);

	//Send Message
	lte_command_response(1,(uint8_t *) publish_cmd_mesg_buffer,
										100, 2, "OK", "ERROR" );
	//printf("\r\n");
	printf("%s\r\n",publish_cmd_mesg_buffer);

//	memset((char*) publish_cmd_mesg_buffer, 0, sizeof(publish_cmd_mesg_buffer));
//	sprintf(publish_cmd_mesg_buffer, "+QMTPUBEX: %d,%d,%s;%s", client_id, mesg_id, bot_id, message);

	return 1;
}


int8_t lte_ntp_sync(void)
{
	uint8_t rx_response;
	rx_response = lte_command_response(1,  (uint8_t *) "at+qntp=1,\"time.windows.com\",123",
										20000, 2, "+QNTP", "ERROR");
	if(1 == rx_response)
	{
		#if WDT
			HAL_IWDG_Refresh(&hiwdg);
		#endif

		return 1;
	}
	return 0;
}


uint8_t	lte_sync_local_time( uint8_t *localTime, int32_t MaxWaiting_ms )
{
	//char localTime[19];
	reset_rx_buffer();
	lte_command_response(0, (uint8_t *)"AT+QLTS=2", 1000, 2, "+QLTS", "ERROR" );
	osDelay(10);

	while(MaxWaiting_ms)
	{
		osDelay(10);
		if(netw_rx_flag == 1)
		{
			char *token = NULL;
			char *rest = NULL;

			token = strtok_r( (char *) uart_input_buffer, "\"", &rest);

			token = strtok_r( NULL, "/", &rest);
			year = (uint8_t) atoi(token) - 2000;

			token = strtok_r( NULL, "/", &rest);
			month = (uint8_t) atoi(token);

			token = strtok_r( NULL, ",", &rest);
			date = (uint8_t) atoi(token);

			token = strtok_r( NULL, ":", &rest);
			hour = (uint8_t) atoi(token);

			token = strtok_r( NULL, ":", &rest);
			min = (uint8_t) atoi(token);

			token = strtok_r( NULL, "+", &rest);
			sec = (uint8_t) atoi(token);

			token = strtok_r( NULL, "\"", &rest);
			timezone = (uint8_t) atoi(token) / 4;

			MX_RTC_Init_Clock();

			sprintf((char *)localTime,"%02d/%02d/%02d | %02d:%02d:%02d",date,month,year,hour,min,sec);
			printf("Local Time: %s\r\n", localTime);
			reset_rx_buffer();
			return 1;
		}
		MaxWaiting_ms -= 10;
	}
	reset_rx_buffer();
	return 0;
}

uint8_t	lte_get_rssi( uint8_t *rssiPtr , int32_t MaxWaiting_ms )
{

	lte_command_response(0, (uint8_t*) "AT+CSQ", 1000, 2, "OK", "ERROR");

	while(MaxWaiting_ms)
	{
		osDelay(10);
		if(netw_rx_flag == 1)
		{
			char *token = NULL;
			char *rest = NULL;

			token = strtok_r( (char *) uart_input_buffer, " ", &rest);

			token = strtok_r( NULL, ",", &rest);
			*rssiPtr = (uint8_t) atoi(token);

			reset_rx_buffer();
			return 1;
		}
		MaxWaiting_ms -= 10;
	}
	reset_rx_buffer();
	return 0;

}

//	Get IP address of network
uint8_t lte_get_ip(void)
{
	uint8_t rx_response;
	rx_response = lte_command_response(0,(uint8_t *) "AT+QIACT?", 10000, 2,
									   "ERROR","\"" );
	if( 1 == rx_response )
	{
		//printf("LTE: Can't Fetch IP\r\n");
		#if WDT
			  HAL_IWDG_Refresh(&hiwdg);
		#endif

		return 0;
	}
	else if( 2 == rx_response )
	{
		char *token = NULL;
		char *rest = NULL;

		token = strtok_r( (char *) uart_input_buffer, "\"", &rest);

		token = strtok_r( NULL, "\"", &rest);
		sprintf(ip_adr,"%s",token);
		reset_rx_buffer();
		printf("LTE\t: Ip fetched : %s \r\n", ip_adr);
		return 1;
	}
	else
	{
		reset_rx_buffer();
		return 0;
	}
}









uint8_t lte_reset(void)
{
	uint8_t rx_response;
//	HAL_Delay(1200);
//	HAL_UART_Transmit ( &gsmHandle , (uint8_t *) "+++" , 3 , HAL_MAX_DELAY );
//	HAL_Delay(1200);
	rx_response = lte_command_response(1, (uint8_t *) "AT+CFUN=1,1", 2000, 1,  "OK");

	if(1 == rx_response)
	{
		#if WDT
			HAL_IWDG_Refresh(&hiwdg);
		#endif
			return 1;
	}
	else
	{
		return 0;
	}

	reset_rx_buffer();

}


//	Connect to GPRS
uint8_t lte_disable_qisend_echo(void)
{
	reset_rx_buffer();
	uint8_t rx_response;
	rx_response = lte_command_response(1,(uint8_t *) "AT+QISDE=0", 5000, 2,
									   "OK", "ERROR" );
	if( 1 == rx_response )
	{
		printf("LTE\t: Echo Disabled\r\n");
		#if WDT
			  HAL_IWDG_Refresh(&hiwdg);
		#endif

		return 1;
	}
	else
	{
		printf("LTE\t: Timeout Disabling echo!\r\n");
	}
	return 0;
}

//	Connect to GPRS
uint8_t lte_connect_to_filepdp(void)
{
	reset_rx_buffer();
	uint8_t rx_response;

	rx_response = lte_command_response(1,(uint8_t *) "AT+QIOPEN=1,2,\"TCP\",\"www.dashboard.greenleaprobotics.com\",80",
			 30000, 2, "+QIOPEN: 2","ERROR");
	if( 1 == rx_response )
		{
			#if WDT
				  HAL_IWDG_Refresh(&hiwdg);
			#endif

			return 1;
		}
	return 0;
}

//	TCP Send Data Command to Connection Number
uint8_t lte_ready_to_send(uint8_t con_number)
{

	uint8_t rx_response;


	//File server
	if(con_number == 0)
	{
		rx_response = lte_command_response(1,(uint8_t *)"AT+QISEND=2", 5000, 2, ">",
			                           "ERROR");
	}
	//MQTT
	else if(1 == con_number)
	{
		rx_response = lte_command_response(1,(uint8_t *)"AT+QISEND=1", 5000, 2, ">",
					                           "ERROR");
	}
	else
	{
		rx_response = lte_command_response(1,(uint8_t *)"AT+QISEND", 5000, 2, ">",
							                           "ERROR");
	}


	if( 1 == rx_response )
	{
		printf("GSM | GPS\t: Ready To Send to %d\r\n", con_number);
		#if WDT
			  HAL_IWDG_Refresh(&hiwdg);
		#endif

		return 1;
	}
	else
	{
		printf("GSM | GPS\t: Something went wrong sending to TCP connection-%d!\r\n", con_number);
		return 0;
	}


}


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


uint8_t check_modem(void)
{

	int8_t attempts = 5;
	while(attempts--)
	{
		if(lte_hard_reset())
			break;

		if(attempts){
			osDelay((5-attempts)*1000);
		}
		else{
			return 0;
		}
	}
	osDelay(1000);
	return 1;
}

uint8_t join_network(void)
{
	lte_echo_mode_off();
	osDelay(100);

	int8_t attempts = 3;
	while(attempts--)
	{
		if(lte_set_service_provider())			//some sims can also connect without reflecting successful APN set
			break;

		if(attempts){
			printf("LTE : Error Setting APN. Retrying...\r\n");
			osDelay((3-attempts)*1000);
		}
	}
	printf("LTE : APN Set.\r\n");
	osDelay(500);

	//if (strcmp(netw_serv_provider, "JIO") == 0)
		//sprintf(MQTTbroker, MQTTbroker_IPv6);

	attempts = 5;
	while(attempts--)
	{
		if(lte_activate_pdp())
			break;

		if(attempts){
			printf("LTE : Error Activating PDP. Retrying...\r\n");
			osDelay(1000);
		}
		else{
			return 0;
		}
	}
	printf("LTE : PDP Activated.\r\n");
	osDelay(100);

	attempts = 3;
	while(attempts--)
	{
		if(lte_mqtt_conf())
			break;

		if(attempts){
			printf("LTE : Error in MQTT configuration. Retrying...\r\n");
			osDelay(1000);
		}
	}
	osDelay(100);

	attempts = 5;
	while(attempts--)
	{
		if(lte_mqtt_open(1,MQTTbroker_IPv4,1883))
		{
			break;
		}

		if(attempts){
			printf("LTE : Error opening MQTT Connection. Retrying...\r\n");
			osDelay(1000);
		}
		else{
			return 0;
		}
	}
	//printf("LTE :  MQTT Connection opened.\r\n");
	osDelay(100);

	attempts = 5;
	while(attempts--)
	{
		if(lte_mqtt_conn(1, (char *)bot_id, USERNAME, PASSWORD))
			break;

		if(attempts){
			printf("LTE : Error connecting MQTT Server. Retrying...\r\n");
			osDelay(1000);
		}
		else{
			return 0;
		}
	}
	printf("LTE : Connected to MQTT Server.\r\n");
	osDelay(100);


	attempts = 5;
	while(attempts--)
	{
		//MQTT Topic: "bot_id/command"
		if(mqtt_subscribe(1, "command", 0)){
			printf("LTE : Subscribed to Commands.\r\n");
			break;
		}

		if(attempts){
			printf("LTE : Error subscribing to Commands. Retrying...\r\n");
			osDelay(1000);
		}

	}
// Re-attempt cmd subscribe to ensure
	osDelay(1000);
	if(mqtt_subscribe(1, "command", 0)){
		printf("LTE : Subscribed to Commands. 2 \r\n");
	}

	return 1;
}


uint8_t sync_time(void)
{
	//get current absolute time and re-Init RTC
	int8_t attempts = 5;
	while(attempts--)
	{
		if(lte_ntp_sync())
			break;

		if(attempts){
			printf("LTE : Unable to connect to NTP server. Retrying...\r\n");
			osDelay(1000);
		}
		else{
			return 0;
		}
	}
	osDelay(100);

	attempts = 5;
	while(attempts--)
	{
		if(lte_sync_local_time(timestampLocal, 2000))
			break;

		if(attempts){
			printf("LTE : Error syncing local time. Retrying...\r\n");
			osDelay(1000);
		}
		else{
			return 0;
		}
	}

	return 1;
}


void get_network_info(void)
{
	lte_get_ip();

	if (!lte_get_rssi(&rssi_current, 1000))
		rssi_current = 0;

	memset(netw_info_buffer, 0, 64);
	sprintf(netw_info_buffer,"RSSI|SP|IP: %d|%s|%s", rssi_current, netw_serv_provider, ip_adr);
}

void post_boot_functions(void)
{
	int8_t attempts = 3;
	//activate gps
	while(attempts--)
	{
		if(lte_activate_gps()){
			gps_start_count = 1;
			break;
		}
		if(attempts){
			printf("LTE : Error activating GPS. Retrying...\r\n");
			osDelay(1000);
		}
	}
	osDelay(500);
}

uint8_t send_state (char* l_dash_status)
{
	memset((char*)msg_packet, 0, sizeof(msg_packet));
	sprintf((char*)msg_packet, "%d;%d;%s;%s;%.1f;%0.0f;%0.0f",
			(int) rssi_current, (int) Battery_SOC, dash_status, gps_adr, CPU_temperature, run_len, total_run_len);
	return mqtt_publish(1, "robot/state", (char *)msg_packet, 0);
}

uint8_t send_response (char* l_dash_response)
{
	return mqtt_publish(1, "robot/response", l_dash_response, 0);
}



#endif /* INC_LTELIB_H_ */
