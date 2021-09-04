/*
 * onenet_config.h
 * 接入参数
 *
 * 
 *
 *
 */


#ifndef _ONENET_CONFIG_H_
#define _ONENET_CONFIG_H_


//设备信息
#define DeviceName           "mytestdev"
#define ProductKey           "a1sYYwS4kmH"
#define DeviceSecret           "8e1833cf4a15218e45338d25fa0ccc78"

//clientID
#define mqttClientId "123|securemode=3,signmethod=hmacsha1|"
//#define mqttClientId-1 "clientId12345deviceNameabcdproductKeya1lZYHY3VzYtimestamp789"


//以下的密码是根据mqttClientId-1和DeviceSecret在网站https://1024tools.com/hmac计算得来
#define mqttPassword          "17225AC3F1B89135E1F193716E99000171405856"



#endif

// 用于模拟器测试
// 阿里云连接域名 a1sYYwS4kmH.iot-as-mqtt.cn-shanghai.aliyuncs.com
