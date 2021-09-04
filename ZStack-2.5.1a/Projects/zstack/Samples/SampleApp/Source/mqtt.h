#ifndef _MQTT_H_
#define _MQTT_H_

#include "mqtt_config.h"
#include "mqttkit.h"


//以下三个值可能是修改,解包的时候使用
#define max_packet_len  300     //封包的最大长度
#define MAX_CMDID_TOPIC_LEN  50   //主题的最大长度 原本50
#define MAX_REQ_PAYLOAD_LEN  100   //内容的最大长度  原本100


_Bool OneNet_DevLink(void);

//void OneNet_SendData(int t, int h);

void OneNet_RevPro(unsigned char *cmd);


/**************************************************************
函数名称 : onenet_mqtt_send_heart
函数功能 : 发送心跳
输入参数 : 无
返回值  	 : 无
备注		 : 无
**************************************************************/
void onenet_mqtt_send_heart(void);


/**************************************************************
函数名称 : onenet_mqtt_publish_topic
函数功能 : onenet mqtt 发布主题消息
输入参数 : topic：发布的主题
		   msg：消息内容
返回值  	 : PUBLISH_MSG_OK: 发布消息成功
		   PUBLISH_MSG_ERROR：发布消息失败
备注		 : 无
**************************************************************/
_Bool mqtt_publish_topic(const char *topic, const char *msg);


/**************************************************************
函数名称 : onenet_mqtt_subscribe_topic
函数功能 : onenet mqtt 订阅主题
输入参数 : topic：订阅的topic
		   topic_cnt：topic个数
返回值  	 : SUBSCRIBE_TOPIC_OK:订阅成功
		   SUBSCRIBE_TOPIC_ERROR：订阅失败
备注		 : 无
**************************************************************/
_Bool mqtt_subscribe_topic(const signed char *topic[], unsigned char topic_cnt);

//发送数据
extern void ESP8266_SendData(char* buff, int len);

//接收到下发命令，或者接收到订阅的消息
extern void mqtt_rx(uint8* topic, uint8* cmd);

extern void debug(uint8 * msg, ...);

#endif
