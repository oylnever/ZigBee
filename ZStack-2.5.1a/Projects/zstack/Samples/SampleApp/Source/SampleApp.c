/*********************************************************************
 * INCLUDES
 */

#include <stdio.h>
#include <string.h>
#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "SampleApp.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "hal_drivers.h"
#include "hal_key.h"
#if defined(LCD_SUPPORTED)
#include "hal_lcd.h"
#endif
#include "hal_led.h"
#include "hal_uart.h"

#ifdef ZDO_COORDINATOR
//Э���ļ�
#include "mqtt.h"
#include "mqttkit.h"

#else

#include "dht11.h"

#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#if !defined(SAMPLE_APP_PORT)
#define SAMPLE_APP_PORT 0
#endif

#if !defined(SAMPLE_APP_RX_MAX)
#define SAMPLE_APP_RX_MAX 200
#endif

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLE_MAX_CLUSTERS] =
    {
        SAMPLEAPP_P2P_CLUSTERID,
        SAMPLEAPP_PERIODIC_CLUSTERID,
        SERIALAPP_CONNECTREQ_CLUSTER,
        SAMPLEAPP_CTE_CLUSTERID // coor to end
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
    {
        SAMPLEAPP_ENDPOINT,             //  int   Endpoint;
        SAMPLEAPP_PROFID,               //  uint16 AppProfId[2];
        SAMPLEAPP_DEVICEID,             //  uint16 AppDeviceId[2];
        SAMPLEAPP_DEVICE_VERSION,       //  int   AppDevVer:4;
        SAMPLEAPP_FLAGS,                //  int   AppFlags:4;
        SAMPLE_MAX_CLUSTERS,            //  byte  AppNumInClusters;
        (cId_t *)SampleApp_ClusterList, //  byte *pAppInClusterList;
        SAMPLE_MAX_CLUSTERS,            //  byte  AppNumOutClusters;
        (cId_t *)SampleApp_ClusterList  //  byte *pAppOutClusterList;
};

endPointDesc_t SampleApp_epDesc =
    {
        SAMPLEAPP_ENDPOINT,
        &SampleApp_TaskID,
        (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc,
        noLatencyReqs};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
devStates_t SampleApp_NwkState;
uint8 SampleApp_TaskID; // Task ID for internal task/event processing.

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 SampleApp_MsgID; //��Ϣid

afAddrType_t SampleApp_P2P_DstAddr; //������

static uint8 SampleApp_RxBuf[SAMPLE_APP_RX_MAX + 1] = {0};
static uint16 SampleApp_RxLen = 0;

uint8 onenet_login_ok = 0; //onenetע��ɹ� 0:δע�ᣬ1��ע���С�2��ע��ɹ�
uint8 end_temp;            //�ն˵��¶�
uint8 end_hum;             //�ն˵�ʪ��

uint8 end_light = 0;     //��������������     ��0-127��
uint8 end_led2 = 0;      //�ն˽ڵ㿪����led3״̬     0�� 1��
uint8 cor_led3 = 0;      //Э����led3 0�� 1��
uint8 end_obstacles = 0; //�ն��ϰ��� 0���ϰ�(����ߵ�ƽ�� 1���ϰ�(�͵�ƽ)
uint8 end_buzzer = 0;    //buzzer 0���� 1����
#ifdef ZDO_COORDINATOR
signed char *g_mqtt_topics_set[5] = {NULL};
u8 topics_buff[60] = {0};
u8 topics_post[100] = {0};  //����������
u8 mqtt_message[200] = {0}; //��������Ϣ
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SampleApp_ProcessMSGCmd(afIncomingMSGPacket_t *pkt);
void SampleApp_CallBack(uint8 port, uint8 event);
static void SampleApp_Send_P2P_Message(void);
static void OneNet_publish_topic();
void debug(uint8 *msg, ...);

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   This is called during OSAL tasks' initialization.
 *
 * @param   task_id - the Task ID assigned by OSAL.
 *
 * @return  none
 */
void SampleApp_Init(uint8 task_id)
{
  halUARTCfg_t uartConfig;

  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;

  MT_UartInit();                  //���ڳ�ʼ��
  MT_UartRegisterTaskID(task_id); //ע�ᴮ������
  afRegister((endPointDesc_t *)&SampleApp_epDesc);
  RegisterForKeys(task_id);

#ifdef ZDO_COORDINATOR
  //Э������ʼ��

  //���������ʼ��

  //  P0SEL &= ~0x80;                 //����P07Ϊ��ͨIO��
  //  P0DIR |= 0x80;                 //P07����Ϊ�����
  //
  //  //Ĭ�Ϸ���������
  //  P0_7=1;

  //��ʼ������������
  sprintf(topics_post, "/post"); //"/sys/%s/%s/thing/event/property/post",ProductKey,DeviceName);

  //��ʼ�����ĵ�����
  sprintf(topics_buff, "/set"); //"/sys/%s/%s/thing/service/property/set",ProductKey,DeviceName);// /sys/a1sYYwS4kmH/mytestdev/thing/service/property/set
  g_mqtt_topics_set[0] = topics_buff;

  //Э�����㲥���ն�
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast; //�㲥ģʽ
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_P2P_DstAddr.addr.shortAddr = 0xFFFF; //�����ն�

  //Ŀ�ĵ�ַ�˿ں�
  endPointDesc_t SampleApp_epDesc;
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;
  afRegister(&SampleApp_epDesc);

#else
  //�ն˵㲥��Э����
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //�㲥
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000; //����Э����
                                                 //��ʼ����ص�ģ�����ӵ����� P0_4_5 ͨ��io ��� ����������
  P0SEL &= 0xCF;                                 //    11001111
  P0DIR |= 0x00;                                 //0_5���룬���ϰ���ʱ������      1���    00010000

#endif
}

void SampleApp_HandleKeys(uint8 shift, uint8 keys)
{
  (void)shift; // Intentionally unreferenced parameter

#if defined(ZDO_COORDINATOR)

  if (keys & HAL_KEY_SW_6) //key1
  {
    const signed char *g_mqtt_topics[] = {"mqtt_topic_test1"};
    if (0 == mqtt_subscribe_topic(g_mqtt_topics, 1))
    {
    }
  }

  if (keys & HAL_KEY_SW_1) //key2
  {
  }

#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events   - Bit map of events to process.
 *
 * @return  Event flags of all unprocessed events.
 */
UINT16 SampleApp_ProcessEvent(uint8 task_id, UINT16 events)
{
  (void)task_id; // Intentionally unreferenced parameter

  if (events & SYS_EVENT_MSG)
  {
    afIncomingMSGPacket_t *MSGpkt;

    while ((MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(SampleApp_TaskID)))
    {
      switch (MSGpkt->hdr.event)
      {
      case KEY_CHANGE:
        SampleApp_HandleKeys(((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys);
        break;

      case AF_INCOMING_MSG_CMD:
        SampleApp_ProcessMSGCmd(MSGpkt);
        break;

      case ZDO_STATE_CHANGE:
        SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
        if ((SampleApp_NwkState == DEV_ZB_COORD) ||
            (SampleApp_NwkState == DEV_ROUTER) || (SampleApp_NwkState == DEV_END_DEVICE))
        {
          //�����ɹ�������һ����ʱ��
          osal_start_timerEx(SampleApp_TaskID,
                             SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                             SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT);
        }
        else
        {
          // Device is no longer in the network
        }
        break;

      default:
        break;
      }

      osal_msg_deallocate((uint8 *)MSGpkt);
    }

    return (events ^ SYS_EVENT_MSG);
  }

#ifdef ZDO_COORDINATOR

  //��ʱ��ʱ�䵽
  if (events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT)
  {
    if (onenet_login_ok == 2)
    {
      //�������onenet�ɹ������������ͷ������ݶ��A��

      P1_1 = 0; //����D2

      // ����������ʱ����15��һ��
      osal_start_timerEx(SampleApp_TaskID, SAMPLEAPP_ONENET_HEART_BEAT_EVT, 15000);

      // �����������ݶ�ʱ����5��һ��
      osal_start_timerEx(SampleApp_TaskID, SAMPLEAPP_ONENET_SEND_DATA_EVT, 5000);
    }
    else
    {
      onenet_login_ok = 1; //onenetע��ɹ� 0:δע�ᣬ1��ע���С�2��ע��ɹ�
      SampleApp_RxLen = 0;

      //���û�н���onenet�ɹ������·������
      OneNet_DevLink(); //����onenet������

      //LED2������ʾ���ڽ���onenet
      HalLedBlink(HAL_LED_2, 5, 50, 500);

      // ÿ5�볢�Խ���һ��
      osal_start_timerEx(SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 5000);
    }

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  //������ʱ��ʱ�䵽
  if (events & SAMPLEAPP_ONENET_HEART_BEAT_EVT)
  {
    if (onenet_login_ok == 2)
    {
      //��������
      onenet_mqtt_send_heart();
    }

    // ����������ʱ����15��һ��
    osal_start_timerEx(SampleApp_TaskID, SAMPLEAPP_ONENET_HEART_BEAT_EVT, 15000);

    // return unprocessed events
    return (events ^ SAMPLEAPP_ONENET_HEART_BEAT_EVT);
  }

  //�������ݶ�ʱ��ʱ�䵽
  if (events & SAMPLEAPP_ONENET_SEND_DATA_EVT)
  {
    if (onenet_login_ok == 2)
    {
      //������ʪ�����ݵ�onenet
      //    OneNet_SendData(end_temp, end_hum);

      sprintf(mqtt_message,
              "{\"method\":\"thing.service.property.set\",\"id\":\"630262306\",\"params\":{\
            \"temp\":%d,\
            \"hum\":%d,\
               \"end_light\":%d,\
              \"end_led2\":%d,\
			  \"end_obstacles\":%d }\
        }",
              end_temp,
              end_hum,
              end_light,
              end_led2,
              end_obstacles);

      //��������
      mqtt_publish_topic(topics_post, mqtt_message);

      //  OneNet_publish_topic();
    }

    // �����������ݶ�ʱ����5��һ��
    osal_start_timerEx(SampleApp_TaskID, SAMPLEAPP_ONENET_SEND_DATA_EVT, 5000);

    // return unprocessed events
    return (events ^ SAMPLEAPP_ONENET_SEND_DATA_EVT);
  }

#else

  //��ʱ��ʱ�䵽
  if (events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT)
  {

    // DHT11�ɼ�
    SampleApp_Send_P2P_Message();

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx(SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                       (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)));

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

#endif

  return (0); // Discard unknown events.
}

/*********************************************************************
 * @fn      SerialApp_ProcessMSGCmd
 *
 * @brief   Data message processor callback. This function processes
 *          any incoming data - probably from other devices. Based
 *          on the cluster ID, perform the intended action.
 *
 * @param   pkt - pointer to the incoming message packet
 *
 * @return  TRUE if the 'pkt' parameter is being used and will be freed later,
 *          FALSE otherwise.
 */
void SampleApp_ProcessMSGCmd(afIncomingMSGPacket_t *pkt)
{
  uint8 buff[20] = {0};

  switch (pkt->clusterId)
  {
  // �����ն��ϴ����¶�����
  case SAMPLEAPP_P2P_CLUSTERID:
#ifdef ZDO_COORDINATOR
  {
    end_temp = pkt->cmd.Data[0]; //�ն��¶�
    end_hum = pkt->cmd.Data[1];  //�ն�ʪ��

    end_light = pkt->cmd.Data[2]; //�ն�����
    end_led2 = pkt->cmd.Data[3];
    end_obstacles = pkt->cmd.Data[4];

    //cor_led3 = !cor_led3;
    sprintf(buff, "T:%d", end_temp);
    HalLcdWriteString(buff, HAL_LCD_LINE_3); //LCD��ʾ

    sprintf(buff, "H:%d", end_hum);
    HalLcdWriteString(buff, HAL_LCD_LINE_4); //LCD��ʾ
  }
#endif
  break;

  case SAMPLEAPP_PERIODIC_CLUSTERID:

    break;
  case SAMPLEAPP_CTE_CLUSTERID: //coor to end �ն˽�����������ʹ���
    char buf[128];
    osal_memset(buf, '\0', 128);
    osal_memcpy(buf, pkt->cmd.Data, pkt->cmd.DataLength); //���Ƶ�������
    HalUARTWrite(0, buf, osal_strlen(buf));               //���������ʾ��Ϣ
    HalUARTWrite(0, "\r\n", 2);                           //���������ʾ��Ϣ

    if (strstr(buf, "D2:0"))
      HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF); //LED2
    else if (strstr(buf, "D2:1"))
      HalLedSet(HAL_LED_2, HAL_LED_MODE_ON); //LED2

    if (strstr(buf, "B:1"))
      P0_4 = 0;
    else if (strstr(buf, "B:0"))
      P0_4 = 1;

    osal_memset(buf, '\0', 128);
    break;
  default:
    break;
  }
}

#ifdef ZDO_COORDINATOR

//����MQTT����
void ESP8266_SendData(char *buff, int len)
{
  if (len == 0)
    return;

  HalUARTWrite(0, buff, len);
}

//���յ��·�������߽��յ����ĵ���Ϣ
//topic:�յ�������
//cmd::�����Ӧ������
void mqtt_rx(uint8 *topic, uint8 *cmd)
{
  char *ptr_led3_on = NULL;
  char *ptr_led3_off = NULL;

  char *this_topic = NULL;
  //���������ʽ
  char uartbuf[] = {'D', '2', ':', ' ',
                    'B', ':', ' ','\0'};
  //Э�����յ����ݺ�
  //������ƾ���������δ���

  if (topic == NULL || cmd == NULL)
    return;

  //����ֻ�������ݣ���ֻ����CMD������������

  ptr_led3_on = strstr((char *)cmd, "\"led\":1");
  ptr_led3_off = strstr((char *)cmd, "\"led\":0");

  if (ptr_led3_on != NULL)
  {
    // coor led3
    //P1_4 = 0; //D3��
    //cor_led3 = 1;
    uartbuf[3] = '1';
    uartbuf[6] = '1';
  }
  else if (ptr_led3_off != NULL)
  {
    // coor led3
    //P1_4 = 1; //D3��
    //cor_led3 = 0;
    uartbuf[3] = '0';
    uartbuf[6] = '0';
  }
  HalLedBlink(HAL_LED_1, 5, 50, 500);

  AF_DataRequest(&SampleApp_P2P_DstAddr,  //&Endevice_Add,  //Ŀ�Ľڵ������ַ�ͷ��͸�ʽ
                 &SampleApp_epDesc,       //Ŀ�ĵ�ַ�˿ں�
                 SAMPLEAPP_CTE_CLUSTERID, //����ţ������������� coor to end
                 7,//osal_strlen(uartbuf),    //�������ݵĳ���
                 uartbuf,                 //�������ݵĻ�����
                 &SampleApp_MsgID,        //�������,����+1
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS);
}

void OneNet_publish_topic()
{
  uint8 buff[20] = {0};

  sprintf(buff, "t:%d,h:%d", end_temp, end_hum);

  /*  ��������Ϊ"hello_topic_public"����ϢΪ�¶Ⱥ�ʪ�� */
  if (0 == mqtt_publish_topic("hello_topic_public", buff))
  {
    ;
  }
}

/*********************************************************************
 * @fn      SampleApp_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
uint32 lastReadMs = 0;
void SampleApp_CallBack(uint8 port, uint8 event)
{
  (void)port;

  if (0 == onenet_login_ok)
  {
    HalUARTRead(SAMPLE_APP_PORT, SampleApp_RxBuf, SAMPLE_APP_RX_MAX);
    SampleApp_RxLen = 0;
    osal_memset(SampleApp_RxBuf, 0, SAMPLE_APP_RX_MAX);
  }
  else if (1 == onenet_login_ok) //onenetע��ɹ� 0:δע�ᣬ1��ע���С�2��ע��ɹ�
  {
    //һ��һ���ֽڶ�
    SampleApp_RxLen += HalUARTRead(SAMPLE_APP_PORT, SampleApp_RxBuf + SampleApp_RxLen, 4);

    //�ж��Ƿ���ע��ɹ�
    if (SampleApp_RxLen >= 4)
    {
      //        debug("rx len=%d(%x,%x,%x,%x)\r\n", SampleApp_RxLen,SampleApp_RxBuf[0],SampleApp_RxBuf[1],SampleApp_RxBuf[2],SampleApp_RxBuf[3]);

      //�ȴ�������Ӧ
      if (MQTT_UnPacketRecv(SampleApp_RxBuf) == MQTT_PKT_CONNACK)
      {
        if (0 == MQTT_UnPacketConnectAck(SampleApp_RxBuf))
        {
          onenet_login_ok = 2; //ע��ɹ�
          SampleApp_RxLen = 0;

          //����������
          if (mqtt_subscribe_topic(g_mqtt_topics_set, 1) == 0)
          {
            //���ĳɹ�P1_4  D3��Ϊ��
            P1_4 = 0;
          }
        }
      }
    }

    //����������0
    if (SampleApp_RxLen >= SAMPLE_APP_RX_MAX)
    {
      SampleApp_RxLen = 0;
    }
  }
  else if (2 == onenet_login_ok)
  {
    //�����ע��ɹ������������Ŀ�������

    uint32 curMs = osal_GetSystemClock();

    if ((curMs - lastReadMs) < 1000)
      return;

    lastReadMs = curMs;
    SampleApp_RxLen = 0;
    osal_memset(SampleApp_RxBuf, 0, SAMPLE_APP_RX_MAX + 1);
    SampleApp_RxLen = HalUARTRead(SAMPLE_APP_PORT, SampleApp_RxBuf, SAMPLE_APP_RX_MAX); //����200������

    if (SampleApp_RxLen > 0)
    {
      debug("rx len=%d.\r\n", SampleApp_RxLen);
      //������Ϣ
      OneNet_RevPro(SampleApp_RxBuf);
      //debug LED3
      //        HalLedBlink (HAL_LED_3, 5, 50, 500); //HalLedSet
      SampleApp_RxLen = 0;
      osal_memset(SampleApp_RxBuf, 0, SAMPLE_APP_RX_MAX + 1);
    }
  }
}

#else
#include "hal_ADC.h"
/*********************************************************************
 * @fn      SampleApp_Send_P2P_Message
 *
 * @brief   point to point.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_Send_P2P_Message(void)
{
  uint8 str[6] = {0};
  uint8 strTemp[20] = {0};
  uint8 end_obstacles_temp = 0;
  uint8 end_buzzer_temp = 0;
  int len = 0;

  osal_memset(str, '\0', 5);
  osal_memset(strTemp, '\0', 20);

  DHT11(); //��ȡ��ʪ��

  HalAdcInit();
  uint8 end_light_temp = HalAdcRead(HAL_ADC_CHANNEL_6, HAL_ADC_RESOLUTION_8);
  uint8 end_led2_temp = 0;

  if (P1_1 == 1)
    end_led2_temp = 0;
  else
    end_led2_temp = 1;

  //�ϰ�����
  if (P0_5 == 1) //����
  {
    end_obstacles_temp = 0;
  }
  else   
    end_obstacles_temp = 1;
  str[0] = wendu; //�¶�
  str[1] = shidu; //ʪ��

  str[2] = end_light_temp;     //����
  str[3] = end_led2_temp;      //led3״̬
  str[4] = end_obstacles_temp; //�ϰ������
  //str[5] = end_buzzer_temp;    //buzzer���
  len = 5;
  str[5] = '\0';
  sprintf(strTemp, "T&H:%d %d", str[0], str[1]);
  HalLcdWriteString(strTemp, HAL_LCD_LINE_3); //LCD��ʾ

  HalUARTWrite(0, strTemp, osal_strlen(strTemp)); //���������ʾ��Ϣ
  HalUARTWrite(0, "\r\n", 2);

  //���߷��͵�Э����
  if (AF_DataRequest(&SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                     SAMPLEAPP_P2P_CLUSTERID,
                     6,//osal_strlen(str),//�����bugosal�ó�33
                     str,
                     &SampleApp_MsgID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SampleApp_CallBack(uint8 port, uint8 event)
{
  (void)port;

  HalUARTRead(SAMPLE_APP_PORT, SampleApp_RxBuf, 1);
}

#endif

//������Ϣ���
void debug(uint8 *msg, ...)
{
#if 0
    uint8 len=0;
    char info[200] = {0};
    va_list args;

    if(msg==NULL) return;

    va_start(args, msg);
    vsprintf(info, (const char*)msg, args);//��ͷ���뵽info��
    va_end(args);
    
    
    len=osal_strlen((char *)info);
    if(len==0) return;
    
    

    HalUARTWrite(1,info, len);//����1����
    osal_memset(info,'\0',200);
//    HAL_UART_PORT_0
#endif
}
