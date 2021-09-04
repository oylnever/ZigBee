/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	onenet.c
	*
	*	作者： 		
	*
	*	日期： 		2017-05-08
	*
	*	版本： 		V1.1
	*
	*	说明： 		与onenet平台的数据交互接口层
	*
	*	修改记录：	V1.0：协议封装、返回判断都在同一个文件，并且不同协议接口不同。
	*				V1.1：提供统一接口供应用层使用，根据不同协议文件来封装协议相关的内容。
	************************************************************
	************************************************************
	************************************************************
**/
//协议文件
#include "mqtt.h"
#include "mqttkit.h"

//C库
#include <string.h>
#include <stdio.h>
#include "hal_led.h"

uint8 packet_buff[max_packet_len]={0};

char cmdid_topic[MAX_CMDID_TOPIC_LEN+1] = {0};
char req_payload[MAX_REQ_PAYLOAD_LEN+1] = {0};

unsigned char mqttUsername[30]={0};
//==========================================================
//	函数名称：	OneNet_DevLink
//
//	函数功能：	与onenet创建连接
//
//	入口参数：	无
//
//	返回参数：	0-成功	1-失败
//
//	说明：		与onenet平台建立连接
//==========================================================
_Bool OneNet_DevLink(void)
{
    MQTT_PACKET_STRUCTURE mqttPacket = {packet_buff, 0, max_packet_len, MEM_FLAG_STATIC};                //协议包

	unsigned char *dataPtr;
	
	_Bool status = 1;

        sprintf(mqttUsername, "%s&%s", DeviceName, ProductKey);
//        debug( "OneNet_DevLink\r\nmqttUsername:%s, mqttPassword:%s,mqttClientId:%s\r\n", mqttUsername, mqttPassword, mqttClientId);
        debug("start mqtt connet!\r\n");
    //接入onenet
	if(MQTT_PacketConnect(mqttUsername, mqttPassword, mqttClientId, 60, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{                                                              //原本的是256为心跳。我修改为60
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);			//上传平台
		
		MQTT_DeleteBuffer(&mqttPacket);								//删包
                debug("mqtt conneted!\r\n");
	}
	else
		debug("WARN:	MQTT_PacketConnect Failed\r\n");
	
	return status;
	
}


//==========================================================
//	函数名称：	OneNet_RevPro
//
//	函数功能：	平台返回数据检测
//
//	入口参数：	dataPtr：平台返回的数据
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void OneNet_RevPro(unsigned char *cmd)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {packet_buff, 0, max_packet_len, 0};								//协议包
	
	unsigned short topic_len = 0;
	unsigned short req_len = 0;

	unsigned char qos = 0;
	static unsigned short pkt_id = 0;

    
	unsigned char type = 0;
	
	short result = 0;
        
	type = MQTT_UnPacketRecv(cmd);
        //解包数据led1闪烁
//        P1_0 = 0; 
//        HalLedBlink (HAL_LED_1, 5, 50, 500);
//        P1_0 = 1;
//        P1_4 = 1; //全灭
//        P1_1 = 1;
//        P1_0 = 1;
	switch(type)
	{       
		case MQTT_PKT_CMD://进不来															//命令下发
    {                   HalLedBlink (HAL_LED_2, 5, 50, 500); //可以进
			result = MQTT_UnPacketCmd(cmd, cmdid_topic, req_payload, &req_len);	//解出topic和消息体
			if(result == 0)
			{
				debug("cmdid: %s, req: %s, req_len: %d\r\n", cmdid_topic, req_payload, req_len);
				mqtt_rx(cmdid_topic, req_payload);
#if 1

				if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)	//命令回复组包
				{
					debug("Tips:	Send CmdResp\r\n");
					
					ESP8266_SendData(mqttPacket._data, mqttPacket._len);			//回复命令
					MQTT_DeleteBuffer(&mqttPacket);									//删包
				}
#endif
			}
    }
		break;
			
		case MQTT_PKT_PUBACK:														//发送Publish消息，平台回复的Ack
		
			if(MQTT_UnPacketPublishAck(cmd) == 0)
				debug("Tips:	MQTT Publish Send OK\r\n");
			
		break;

        case MQTT_PKT_PINGRESP:
        {
            debug("Tips:  HeartBeat OK\r\n");
            break;
        }
        
        //进不来   
        case MQTT_PKT_PUBLISH:  //接收的Publish消息
        {
//            HalLedBlink (HAL_LED_3, 5, 50, 500); //也可以进，要注意数据长度
            if(MQTT_UnPacketPublish(cmd, cmdid_topic, &topic_len, req_payload, &req_len, &qos, &pkt_id) == 0)
            {
                debug("topic: %s, topic_len: %d, payload: %s, payload_len: %d\r\n",cmdid_topic, topic_len, req_payload, req_len);
                mqtt_rx(cmdid_topic, req_payload);

               // MQTT_FreeBuffer(cmdid_topic);
               // MQTT_FreeBuffer(req_payload);        

                switch(qos)
                {
                    case 1:                                                         //收到publish的qos为1，设备需要回复Ack
                    {
                        if(MQTT_PacketPublishAck(pkt_id, &mqttPacket) == 0)
                        {
                            debug("Tips:  Send PublishAck\r\n");
                            ESP8266_SendData(mqttPacket._data, mqttPacket._len);
                            MQTT_DeleteBuffer(&mqttPacket);
                        }
                        break;
                    }
                    case 2: //收到publish的qos为2，设备先回复Rec
                    {       //平台回复Rel，设备再回复Comp
                        if(MQTT_PacketPublishRec(pkt_id, &mqttPacket) == 0)
                        {
                            debug("Tips:  Send PublishRec\r\n");
                            ESP8266_SendData(mqttPacket._data, mqttPacket._len);
                            MQTT_DeleteBuffer(&mqttPacket);
                        }
                        break;
                    }
                    default:
                        break;
                }
                
            }
            break;
        }
        case MQTT_PKT_PUBREC:   //发送Publish消息，平台回复的Rec，设备需回复Rel消息
        {
            if(MQTT_UnPacketPublishRec(cmd) == 0)
            {
                debug("Tips:  Rev PublishRec\r\n");
                if(MQTT_PacketPublishRel(MQTT_PUBLISH_ID, &mqttPacket) == 0)
                {
                    debug("Tips:  Send PublishRel\r\n");
                    ESP8266_SendData(mqttPacket._data, mqttPacket._len);
                    MQTT_DeleteBuffer(&mqttPacket);
                }
            }
            break;
        }   
        case MQTT_PKT_PUBREL://收到Publish消息，设备回复Rec后，平台回复的Rel，设备需再回复Comp
        {
            if(MQTT_UnPacketPublishRel(cmd, pkt_id) == 0)
            {
                debug("Tips:  Rev PublishRel\r\n");
                if(MQTT_PacketPublishComp(MQTT_PUBLISH_ID, &mqttPacket) == 0)
                {
                   debug("Tips:  Send PublishComp\r\n");
                    ESP8266_SendData(mqttPacket._data, mqttPacket._len);
                    MQTT_DeleteBuffer(&mqttPacket);
                }
            }
            break;
        }
        case MQTT_PKT_PUBCOMP:  //发送Publish消息，平台返回Rec，设备回复Rel，平台再返回的Comp
        {
            if(MQTT_UnPacketPublishComp(cmd) == 0)
            {
                debug("Tips:  Rev PublishComp\r\n");
            }
            break;
        }
        case MQTT_PKT_SUBACK:   //发送Subscribe消息的Ack
        {
            if(MQTT_UnPacketSubscribe(cmd) == 0)
            {
                debug("Tips:  MQTT Subscribe OK\r\n");
            }
            else
            {
                debug("Tips:  MQTT Subscribe Err\r\n");
            }
            break;
        }
        case MQTT_PKT_UNSUBACK: //发送UnSubscribe消息的Ack
        {
            if(MQTT_UnPacketUnSubscribe(cmd) == 0)
            {
                debug("Tips:  MQTT UnSubscribe OK\r\n");
            }
            else
            {
                debug("Tips:  MQTT UnSubscribe Err\r\n");
            }
            break;
        } 		
		default:
			result = -1;
		break;
	}
}

/**************************************************************
函数名称 : onenet_mqtt_send_heart
函数功能 : 发送心跳
输入参数 : 无
返回值  	 : 无
备注		 : 无
**************************************************************/
void onenet_mqtt_send_heart(void)
{
    MQTT_PACKET_STRUCTURE mqttPacket = {packet_buff, 0, max_packet_len, MEM_FLAG_STATIC};                //协议包

	debug("send heart\r\n");
	MQTT_PacketPing(&mqttPacket);
    ESP8266_SendData(mqttPacket._data, mqttPacket._len);
	MQTT_DeleteBuffer(&mqttPacket);									//删包
}


/**************************************************************
函数名称 : mqtt_publish_topic
函数功能 : onenet mqtt 发布主题消息
输入参数 : topic：发布的主题
		   msg：消息内容
返回值  	 : PUBLISH_MSG_OK: 发布消息成功
		   PUBLISH_MSG_ERROR：发布消息失败
备注		 : 无
**************************************************************/
_Bool mqtt_publish_topic(const char *topic, const char *msg)
{
  MQTT_PACKET_STRUCTURE mqttPacket = {packet_buff, 0, max_packet_len, MEM_FLAG_STATIC};                //协议包
	
	debug("publish topic: %s, msg: %s\r\n", topic, msg);

	if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic, msg, strlen(msg), MQTT_QOS_LEVEL2, 0, 1, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);//上传平台
		MQTT_DeleteBuffer(&mqttPacket);	//删包
		return 0;
	}
	else
	{
		return 1;
	}

}

/*  发布主题为"hello_topic"，消息为"hello_world" */
//if(0 == onenet_mqtt_publish_topic("hello_topic", "hello_world"))
//{;}

 
/**************************************************************
函数名称 : onenet_mqtt_subscribe_topic
函数功能 : onenet mqtt 订阅主题
输入参数 : topic：订阅的topic
		   topic_cnt：topic个数
返回值  	 : SUBSCRIBE_TOPIC_OK:订阅成功 0
		   SUBSCRIBE_TOPIC_ERROR：订阅失败 其他
备注		 : 无
**************************************************************/
_Bool mqtt_subscribe_topic(const signed char *topic[], unsigned char topic_cnt)
{
	
	unsigned char i = 0;
  MQTT_PACKET_STRUCTURE mqttPacket = {packet_buff, 0, max_packet_len, MEM_FLAG_STATIC};                //协议包
 
	for(i=0; i < topic_cnt; i++)
	{
		debug("subscribe topic: %s\r\n", topic[i]);
	}
    
	if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL2, topic, topic_cnt, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);//上传平台
		MQTT_DeleteBuffer(&mqttPacket);											//删包
		return 0;
	}
	else
	{
		return 1;
	}
}

//主题订阅方法
//const signed char *g_mqtt_topics[] = {"mqtt_topic", "topic_test"};
//if(0 == onenet_mqtt_subscribe_topic(g_mqtt_topics, 2))



