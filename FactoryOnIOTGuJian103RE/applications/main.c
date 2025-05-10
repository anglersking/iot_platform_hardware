
#include <drivers/pin.h>
#include <finsh_api.h>
#include <rtdef.h>
#include <rthw.h>
#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
/*-------------------------------------------------*/
#include "dhtxx.h"
#include <at.h>   /* AT 组件头文件 */
#include <at_device_ec20.h>
#include "rt_cjson_tools.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_conf.h"

#include <rtdevice.h>
#include "fal.h"
/*-------------------adc------------------------------*/
#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define ADC_DEV_CHANNEL      10           /* ADC 通道-pc0为通道10 */
#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

/*-------------------adc------------------------------*/
#define APP_VERSION             "1.0.6"
#define DATA_PIN                 PKG_USING_DHTXX_SAMPLE_PIN

#define BEEPPIN                 38  /* Pc6 -蜂鸣器*/
#define KEY1                    29  /* Pb13 -设备类型检测引脚key1,仅1闭合的时候是A，仅2闭合是B，1和2都闭合是C*/
#define KEY2                    30  /* Pb14 -设备类型检测引脚key1*/
#define GRESET                  31  /* Pb15 -4g模块的复位引脚*/
#define userkey                 39  /* Pc7 -userkey*/
/****for A & B device *****/
#define AD3BD8                  42  /* Pc10 */
#define AD7BD7                  43  /* Pc11 */
#define AD6BD5                  44  /* Pc12 */
#define AD2BD6                  50  /* PD2 */
#define AD4BD4                  20  /* Pb4 */

#define AD1BD3                  21 /*PB5-*/ //!!!!这里需要注意，如果使用上面的一根线代替，这里需要注意电平设置或者硬件断开。以免影响
#define AD5BD1                  22 /*PB6*/
#define AD8BD2                  23 /*pb7*/

#define CONTROSELECT            12  /* Pa12 -A和B设备的3路控制信号通道选择芯片CD4053的选择引脚，高电平时候,三种信号STB,CLOCK,OUTPUTENABLE都连通B设备的3路信号，低电平的时候三路信号连通到A设备*/
#define ABCLOCKOUTPUT           24  /*pb8- A和B设备的输出clock控制信号*/
#define ABSTBOUTPUT             11  /*pa11- A和B设备的输出stobe控制信号*/
#define ABOUTPUTENABLEOUTPUT    25  /*pb9- A和B设备的输出OUTPUTENABLE控制信号*/
#define AB4245ENABLEOUTPUT      15 /*pa15- A和B设备的数据输出控制信号*/
/****for A device only*****/
#define AP111IN                 41  /*pc9- A设备的P1-11信号,这个是输入信号*/
#define AComModelEnable         40  /*pc8- A设备的公共模块ENABLE控制信号*/
#define ADeviceSelSet           8  /*pa8- A设备的新主板选择*/
/****for B device only*****/
#define BComModelEnable         46 /*pc14- B设备的公共模块ENABLE控制信号*/
#define BDeviceSelSet           45 /*pc13- B设备的公共模块ENABLE控制信号*/
#define BCPU64IN                47 /*pc15- B设备的公共模块ENABLE控制信号*/
/****for C device only*****/
#define C8STATIONDATAOUTPUT     17  /* Pb1 -C设备的8站模块输出数据线*/
#define CComModelEnable         16  /* Pb0 -C设备的公共模块输出使能线*/

#define C4245ENABLE             46 /*/*pc14- 公用 B设备的公共模块ENABLE控制信号 -C设备的隔离芯片74LVX4245的使能端，当其为低的时候，数据才能传输，此行以下的6个引脚数据才能传输过去！！！*/
#define C138A2OUTPUT            35 /*Pc3 -74hc138的选通信号的地址脚，确定6路中哪一路信号为低电平*/
#define C138A1OUTPUT            0 /*Pa0 -74hc138的选通信号的地址脚，确定6路中哪一路信号为低电平*/
#define C138A0OUTPUT            1 /*Pa1 -74hc138的选通信号的地址脚，确定6路中哪一路信号为低电平*/
#define C138CLEAR               34/*pc2-74hc138的全部拉高引脚*/
#define C8STATIONCLROUTPUT      33 /*Pc1 -C设备的8站模块的清除引脚*/
#define C8STATIONADDRESSOUTPUT  36  /*pc4-C设备的地址数据输出引脚，尽量使用这里的引脚（而不是以下的三路信号分别输出）输出到161计数器，产生C8STATIONA0-A3的3路地址数据*/
#define C74HC161MROUTPUT         2  /*Pa2-161计数器的清零*/
#define CDeviceSelOUTPUT        3 /*Pa3-c新主板的选择引脚*/
#define CBUSDETECIN             37 /*Pc5-总线的监测引脚*/

#define WDT_DEVICE_NAME    "wdt"    /* 看门狗设备名称 */
static rt_device_t wdg_dev;         /* 看门狗设备句柄 */
rt_uint32_t timeout = 25;       /* 溢出时间，单位：秒,最高不能超过26s*/

int DeviceType =0 ;         //设备类型的标记，选择A设备时候=1；B设备时候=2；C设备时候=3
int Device8StationsNum = 0 ;
int Device8StationsNum2 = 0 ;
int AllStationOFF = 1;//如果所有的站点都关闭了，则这个为0
int intqmtrurc_func = 0;
int errcode = 0;
int NetConnected = 0;//指示是否在联网状态，如果网络在线的话，NetConnected = 0，掉线的话NetConnected = 1
int MQTTConnected = 0;//指示是mqtt的联网状态，如果mqtt在线的话，MQTTConnected = 0，掉线的话MQTTConnected = 1
int OperatorCode = 0; //指示网络运营商的类别，因为三个运营商的物联网卡，联通的和电信的最后一位都是字母，而中国移动的最后一位很少是字母。如果是中国移动则为1，其他两家为0，为0则去掉最后一位

int ADeviceStatus[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};/*A，B,C设备的所有站点的状态，每个bit代表一个站点的状态，0代表OFF，1代表的是ON*/
int BDeviceStatus[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int CDeviceStatus[]={0x00,0x00,0x00,0x00,0x00,0x00};
int ADevicePins[]={21,50,42,20,22,44,43,23};
int BDevicePins[]={22,23,21,20,44,50,43,42};
int CDevicePins[]={0x00,0x00,0x00,  0x01,0x00,0x00,  0x00,0x01,0x00,  0x01,0x01,0x00,  0x00,0x00,0x01,  0x01,0x00,0x01};/*C设备通过138对每个八站的片选电平*/
int ADeviceStatusTemp[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int BDeviceStatusTemp[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int CDeviceStatusTemp[] = {0x00,0x00,0x00,0x00,0x00,0x00};

int ADevice8StationSelectedNum =0; //通讯的时候依据stationNum判断出来的，位于第几个八站模块中；
int BDevice8StationSelectedNum =0; //通讯的时候依据stationNum判断出来的，位于第几个八站模块中；
int CDevice8StationSelectedNum =0; //通讯的时候依据stationNum判断出来的，位于第几个八站模块中；
int TimeToRemain[64] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
rt_uint32_t  volvalue = 0;
rt_uint32_t  ia = 0;
struct rt_alarm * alarm = RT_NULL;
/*--------------------以下为MQTT连接需要使用的变量---------------------------*/
char device_id[30] = {0};
char device_id2[30] = {0};
char device_id3[30] = {0};

char *cliend_idx = "0";
char receivecliend_idx[1]= {0};

char *messageid ="1";
char *QOS0messageid ="1";//当QOS=0的时候，msgid就只能是0
char receivemessageid[6]= {0};
char receivoperationid[30]= {0};
int  receivruntime= 0;

char *qos ="1";
char receiveqos[]= {0};

char *message ="IsOnLine!";
char receivemessage[500]= {0};
char str[500]={0};
char messagelength[]= {0};
char type[]= {0};
char num[]= {0};


char *retain = "0";
char *topic ="\"test\"";/*1*/
char *sendtopic ="\"test\"";/*2*/
char *recvtopic ="\"test\"";/*1*/
char receivetopic[10]= {0};
int ret =0;
int retsum = 0;
float temperature;
float humidity;
int signalstrength[3] = {0};
int biterrorrate;
/*--------------------以下连接到云服务器的常用命令---------------------------*/
char *QMTOPENcmd = "AT+QMTOPEN";
char *QMTCLOSEcmd = "AT+QMTCLOSE";
char *QMTCONNcmd = "AT+QMTCONN";
char *QMTDISCcmd = "AT+QMTDISC";
char *QMTSUBcmd = "AT+QMTSUB";
char *QMTUNScmd = "AT+QMTUNS";
char *QMTPUBEXcmd = "AT+QMTPUBEX";
char *point = ",";
char *equals = "=";
/*--------------------以下连接到云服务器的一些参数---------------------------*/
/*char *host_name ="\"post-cn-i7m2fwizj12.mqtt.aliyuncs.com\"";     这个是可用的
char *port = "1883";*/

char *host_name ="\"post-cn-omn3o8zvu01.mqtt.aliyuncs.com\"";
char *port = "1883";

/*char *clientid ="\"GID_test@@@3\"";
char *Username ="\"Signature|LTAI5t6E7XoRLUt4ewRupXA5|post-cn-i7m2fwizj12\"";
char *Password ="\"jtAlQh0OLLr8y4IkOmXAjqgBWv8=\"";*/

/*char *clientid ="\"GID_test@@@2\"";                                这个是可用的
char *Username ="\"Signature|LTAI5t6E7XoRLUt4ewRupXA5|post-cn-i7m2fwizj12\"";
char *Password ="\"d5vqVLKa3cW+ngDuru69jCotxqY=\"";*/

char *clientid ="\"GID_001@@@test_topic\"";
char *Username ="\"Signature|LTAI5tJeLqp6F7DvqVxJiqpu|post-cn-omn3o8zvu01\"";
char *Password ="\"WY2pdjSPiDl6Q3WkN6i6jP5y7W3zY1\"";



/*下面是使用张总的账户了--------------------------------------------------------------------------------------*/
//char *host_name ="\"post-cn-tl32m9urp0e.mqtt.aliyuncs.com\"";

/*char *clientid ="\"GID_test@@@3\"";
char *Username ="\"Signature|LTAI5t8DoNt9hijy4g8WovbX|post-cn-tl32m9urp0e\"";
char *Password ="\"qF0YmQM7prFM2Cl2U/Edk3v4tuw=\"";*/

/*char *clientid ="\"GID_test@@@1\"";
char *Username ="\"Signature|LTAI5t8DoNt9hijy4g8WovbX|post-cn-tl32m9urp0e\"";
char *Password ="\"gbr3EoWVKUy/ZIECeN5vqBOfVrk=\"";*/

/*--------------------以上连接到云服务器的一些参数---------------------------*/
/* cat_dhtxx sensor data by dynamic */
static int adc_vol_sample()
{
    rt_adc_device_t adc_dev;
    rt_uint32_t value, vol;
    rt_err_t ret = RT_EOK;
    rt_uint32_t maxvalue,maxvol;
//    fal_init();
    /* 查找设备 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 使能设备 */
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);

    /* 读取采样值 */
    value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
    rt_kprintf("the value is :%d \n", value);

    maxvalue = 0;
    for (int var = 0; var < 60; var++) {
        value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
        if ( value > maxvalue) {
            maxvalue = value;
        }
        rt_thread_mdelay(1);
    }
    rt_kprintf("the maxvalue is :%d \n", maxvalue);
    maxvol = maxvalue * REFER_VOLTAGE / CONVERT_BITS;
    volvalue = maxvol;
    rt_kprintf("the maxvoltage is :%d.%02d \n", maxvol / 100, maxvol % 100);

    /* 转换为对应电压值 */
   /* vol = value * REFER_VOLTAGE / CONVERT_BITS;
    rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);
    volvalue = vol;
*/
    /* 关闭通道 */
    ret = rt_adc_disable(adc_dev, ADC_DEV_CHANNEL);

    return ret;
}
static void cat_dhtxx(void)
{
    dht_device_t sensor = dht_create(DATA_PIN);
    rt_thread_mdelay(2000);
    if(dht_read(sensor)) {

        rt_int32_t temp = dht_get_temperature(sensor);
        rt_int32_t humi = dht_get_humidity(sensor);

        rt_kprintf("Temp: %d, Humi: %d\n", temp, humi);
    }
    else {
        rt_kprintf("Read dht sensor failed1.\n");
    }
    dht_delete(sensor);
}

#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(cat_dhtxx, read dhtxx humidity and temperature);
#endif

/*蜂鸣器响times次，每次响声持续length毫秒,每次响声之间间隔interval 毫秒*/
void BEEP(int times,int length,int interval)
{
    for (int var = 0; var < times; ++var) {
        rt_pin_write(BEEPPIN, PIN_HIGH);
        rt_thread_mdelay(length);
        rt_pin_write(BEEPPIN, PIN_LOW);
        rt_thread_mdelay(interval);
    }
}

/*检测安装的八站模块的的中断的回调函数，用来计数*/
void CountABCDevice8StationsNum(void)
{
    Device8StationsNum ++ ;
}

/*设置A和B设备通讯时候使用的一些引脚的模式*/
int SetABDevicePins(void)
{
    /*设定A和B设备的输出数据线 ，设定为输出模式*/
    rt_pin_mode(AD3BD8, PIN_MODE_OUTPUT);
    rt_pin_mode(AD7BD7, PIN_MODE_OUTPUT);
    rt_pin_mode(AD6BD5, PIN_MODE_OUTPUT);
    rt_pin_mode(AD2BD6, PIN_MODE_OUTPUT);
    rt_pin_mode(AD4BD4, PIN_MODE_OUTPUT);
    rt_pin_mode(AD1BD3, PIN_MODE_OUTPUT);
    rt_pin_mode(AD5BD1, PIN_MODE_OUTPUT);
    rt_pin_mode(AD8BD2, PIN_MODE_OUTPUT);

    /*A和B设备的3路控制信号通道选择芯片CD4053的选择引脚，高电平时候,指向B设备；低电平时候，指向A设备*/
    rt_pin_mode(CONTROSELECT, PIN_MODE_OUTPUT);

    /*A和B设备的输出clock控制信号，输出stobe控制信号，输出OUTPUTENABLE控制信号*/
    rt_pin_mode(ABCLOCKOUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(ABSTBOUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(ABOUTPUTENABLEOUTPUT, PIN_MODE_OUTPUT);
    /*A和B设备的主板输入信号，用于接受下面八站模块的信号*/
    rt_pin_mode(AP111IN, PIN_MODE_INPUT);
    rt_pin_mode(BCPU64IN , PIN_MODE_INPUT);
    /*A和B设备的八站模块的控制信号*/
    rt_pin_mode(AComModelEnable, PIN_MODE_OUTPUT);
    rt_pin_mode(BComModelEnable, PIN_MODE_OUTPUT);
    /*A和B设备的数据输出的控制信号，高电平有效，会使74lvx4245芯片导通，数据才可以传输过去，默认应该为低*/
    rt_pin_mode(AB4245ENABLEOUTPUT, PIN_MODE_OUTPUT);
    /*A和B设备的数据新旧主板数据通路选择控制信号，高电平有效，会使74cbt3257芯片使能，数据通道切换到新主板，默认是切换到旧主板上*/
    rt_pin_mode(ADeviceSelSet, PIN_MODE_OUTPUT);
    rt_pin_mode(BDeviceSelSet, PIN_MODE_OUTPUT);
    return RT_EOK;
}

/*设置C设备通讯时候使用的一些引脚的模式*/
int SetCDevicePins(void)
{
    rt_pin_mode(C8STATIONDATAOUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(CComModelEnable, PIN_MODE_OUTPUT);

    rt_pin_mode(C4245ENABLE, PIN_MODE_OUTPUT);
    rt_pin_mode(C138A2OUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(C138A1OUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(C138A0OUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(C138CLEAR, PIN_MODE_OUTPUT);

    rt_pin_mode(C8STATIONCLROUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(C8STATIONADDRESSOUTPUT, PIN_MODE_OUTPUT);
    /*计数器161的清零引脚，*/
    rt_pin_mode(C74HC161MROUTPUT, PIN_MODE_OUTPUT);
    rt_pin_mode(CDeviceSelOUTPUT, PIN_MODE_OUTPUT);
    /*设定为输入模式*/
    rt_pin_mode(CBUSDETECIN, PIN_MODE_INPUT_PULLDOWN);


    /*设置74LVX4245的22脚enable为低电平，设定传输方向为B 到 A*/
    rt_pin_write(C4245ENABLE, PIN_LOW);
    return RT_EOK;
}

/*通过key1和2检测出设备类型，未选择的时候为错误，返回1*/
int DetectDeviceType(void)
{
    rt_pin_mode(KEY1, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KEY2, PIN_MODE_INPUT_PULLUP);
    if((rt_pin_read(KEY1)==PIN_LOW)&&(rt_pin_read(KEY2)==PIN_HIGH))
    {
        LOG_D("chose a device!");
       DeviceType = 1;
        return RT_EOK;
    }else if ((rt_pin_read(KEY1)==PIN_HIGH)&&(rt_pin_read(KEY2)==PIN_LOW)) {
        LOG_D("chose b device!");
        DeviceType = 2;
        return RT_EOK;
    }else if ((rt_pin_read(KEY1)==PIN_LOW)&&(rt_pin_read(KEY2)==PIN_LOW)) {
        LOG_D("chose c device!");
        DeviceType = 3;
        return RT_EOK;
    }else{
        LOG_D("err!");
        return RT_ERROR;
    }
}

/*根据拨码开关中判断的设备类型，上电的时候进行检测，返回所选设备下面所接上的八站模块的个数*/
int DetectDevice8StationNum(int DeviceType)
{
    if (DeviceType == 1) {
        int BitMask = 0x80;
        int Atemp = 0x80;
        /*使能新主板的数据通讯*/
        rt_pin_write(ADeviceSelSet, PIN_HIGH);
        rt_pin_write(AB4245ENABLEOUTPUT, PIN_HIGH);
        /*开始计数前将计数值清零*/
        Device8StationsNum = 0;
        rt_pin_mode(AP111IN, PIN_MODE_INPUT_PULLDOWN);
        /* 绑定中断，上升沿模式，回调函数名为CountABCDevice8StationsNum */
        rt_pin_attach_irq(AP111IN, PIN_IRQ_MODE_RISING, CountABCDevice8StationsNum, RT_NULL);
        /* 使能中断 */
        rt_pin_irq_enable(AP111IN, PIN_IRQ_ENABLE);
        //下面是将控制信号及时钟信号拉低，将所有的数据线拉低
        rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_LOW);
        rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
        rt_pin_write(ABSTBOUTPUT, PIN_LOW);
        for (int var = 0; var < 8; var++) {
            rt_pin_write(ADevicePins[var], PIN_LOW);
        }
        rt_hw_us_delay(40);
        //下面是开始发送八位个clk，八位数据全部为零（参照旧主板的动作）且不使能动作，这样的话继电器不会有不期望执行的动作
        for (int var = 0; var < 8; var++) {
            rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
            rt_hw_us_delay(100);
            rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
            rt_hw_us_delay(100);
        }
        //下面STB的一段高电平动作
        rt_pin_write(ABSTBOUTPUT, PIN_HIGH);
        rt_pin_write(ABSTBOUTPUT, PIN_LOW);
        rt_hw_us_delay(30);
        /*八个八站模块的循环*/
        for (int var1 = 0; var1 < 8; var1++) {
            for (int var = 0; var < 8; var++) { //每个八站传输八位数据，其中高位的数据先传输出去，但是不使能动作
                rt_hw_us_delay(50);
                ((Atemp & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[var1], PIN_HIGH) : rt_pin_write(ADevicePins[var1], PIN_LOW);
                rt_hw_us_delay(200);
                rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
                rt_hw_us_delay(200);
                /*时钟的速度不能过快，不然无法检测出背面的八站模块数量*/
                rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
                rt_hw_us_delay(200);
                /*无论数据是1还是0，这里都是拉低*/
                rt_pin_write(ADevicePins[var1], PIN_LOW);
                Atemp >>= 1;
            }
            Atemp = 0x80;
        }
        LOG_D("A--DetectDevice8StationNum-----Device8StationsNum = %d",Device8StationsNum);
        /*失去能新主板的数据通讯*/
        rt_pin_write(ADeviceSelSet, PIN_LOW);
        rt_pin_write(AB4245ENABLEOUTPUT, PIN_LOW);
        return Device8StationsNum;
    }else if (DeviceType == 2) {
       int BitMask = 0x80;
       int Atemp = 0x80;
       /*使能新主板的数据通讯*/
       rt_pin_write(BDeviceSelSet, PIN_HIGH);
       rt_pin_write(AB4245ENABLEOUTPUT, PIN_HIGH);
       /*开始计数前将计数值清零*/
       Device8StationsNum = 0;
       rt_pin_mode(BCPU64IN, PIN_MODE_INPUT_PULLDOWN);
       /* 绑定中断，上升沿模式，回调函数名为CountABCDevice8StationsNum */
       rt_pin_attach_irq(BCPU64IN, PIN_IRQ_MODE_RISING, CountABCDevice8StationsNum, RT_NULL);
       /* 使能中断 */
       rt_pin_irq_enable(BCPU64IN, PIN_IRQ_ENABLE);
       //下面是将控制信号及时钟信号拉低，将所有的数据线拉低
       rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_LOW);
       rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
       rt_pin_write(ABSTBOUTPUT, PIN_LOW);
           for (int var = 0; var < 8; var++) {
               rt_pin_write(BDevicePins[var], PIN_LOW);
           }
       rt_hw_us_delay(40);
       //下面是开始发送八位个clk，八位数据全部为零（参照旧主板的动作）且不使能动作，这样的话继电器不会有不期望执行的动作
       for (int var = 0; var < 8; var++) {
          rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
          rt_hw_us_delay(100);
          rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
          rt_hw_us_delay(100);
       }
       //下面STB的一段高电平动作
       rt_pin_write(ABSTBOUTPUT, PIN_HIGH);
       rt_pin_write(ABSTBOUTPUT, PIN_LOW);
       rt_hw_us_delay(30);
       /*八个八站模块的循环*/
        for (int var1 = 0; var1 < 8; var1++) {
          for (int var = 0; var < 8; var++) { //每个八站传输八位数据，其中高位的数据先传输出去，但是不使能动作
              rt_hw_us_delay(2);
             ((Atemp & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[var1], PIN_HIGH) : rt_pin_write(BDevicePins[var1], PIN_LOW);
             rt_hw_us_delay(10);
             rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
             rt_hw_us_delay(100);
             /*时钟的速度不能过快，不然无法检测出背面的八站模块数量*/
             rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
             rt_hw_us_delay(150);
             /*无论数据是1还是0，这里都是拉低*/
             rt_pin_write(BDevicePins[var1], PIN_LOW);
             Atemp >>= 1;
         }
          Atemp = 0x80;
      }
      LOG_D("B--DetectDevice8StationNum-----Device8StationsNum = %d",Device8StationsNum);
       /*失去能新主板的数据通讯*/
       rt_pin_write(BDeviceSelSet, PIN_LOW);
       rt_pin_write(AB4245ENABLEOUTPUT, PIN_LOW);
       return Device8StationsNum;

    }else if (DeviceType == 3){
        /*开始计数前将计数值清零*/
        Device8StationsNum = 0;
        /*使能新主板的数据通讯*/
        rt_pin_write(C4245ENABLE, PIN_LOW);
        rt_pin_write(CDeviceSelOUTPUT, PIN_HIGH);

        /*设定为输入模式*/
        rt_pin_mode(CBUSDETECIN, PIN_MODE_INPUT_PULLUP);
        /* 绑定中断，下降沿模式，回调函数名为CountABCDevice8StationsNum */
        rt_pin_attach_irq(CBUSDETECIN, PIN_IRQ_MODE_FALLING, CountABCDevice8StationsNum, RT_NULL);
        /* 使能中断 */
        rt_pin_irq_enable(CBUSDETECIN, PIN_IRQ_ENABLE);
        /*设置74LVX4245的22脚enable为低电平，设定传输方向为B 到 A*/
         rt_pin_write(C4245ENABLE, PIN_LOW);
         /*设置clr引脚为高电平，各个八站可以正常接受数据并动作，如果是低电平则会复位。这里需要设置为高,不然在发送数据之前，无法检测出来安装了几个八站模块*/
         rt_pin_write(C8STATIONCLROUTPUT,PIN_HIGH);

        for (int var = 0; var < 6; var++) {
           rt_pin_irq_enable(CBUSDETECIN, PIN_IRQ_ENABLE);
           rt_pin_write(C138CLEAR, PIN_LOW);
           rt_pin_write(C138A0OUTPUT, CDevicePins[(var*3)]);
           rt_pin_write(C138A1OUTPUT, CDevicePins[(var*3)+1]);
           rt_pin_write(C138A2OUTPUT, CDevicePins[(var*3)+2]);
           rt_pin_write(C138CLEAR, PIN_HIGH);
           rt_hw_us_delay(50);
           //再快速将所有的与主板相连的引脚都拉高，全部不选
           rt_pin_write(C138CLEAR, PIN_LOW);
        }
        LOG_D("DetectDevice8StationNum-----Device8StationsNum = %d",Device8StationsNum);
        /*失去能新主板的数据通讯*/
         rt_pin_write(CDeviceSelOUTPUT, PIN_LOW);
         return Device8StationsNum;
    }
}

/*依据A设备的及站编号，算出站位于哪个八站模块中*/
int SetADeviceDataChannel(int DeviceType,int StationNum)
{
    if (DeviceType == 1) {
        switch (StationNum) {
            case 1 ... 8:
            ADevice8StationSelectedNum =1;
                break;
            case 9 ... 16:
            ADevice8StationSelectedNum =2;
                break;
            case 17 ... 24:
            ADevice8StationSelectedNum =3;
                break;
            case 25 ... 32:
            ADevice8StationSelectedNum =4;
                break;
            case 33 ... 40:
            ADevice8StationSelectedNum =5;
                break;
            case 41 ... 48:
            ADevice8StationSelectedNum =6;
                break;
            case 49 ... 56:
            ADevice8StationSelectedNum =7;
                break;
            case 57 ... 64:
            ADevice8StationSelectedNum =8;
                break;

            default:
                break;
        }
        return RT_EOK;

    }else {
        return RT_ERROR;
    }
}

/*依据B设备的站编号，算出站位于哪个八站模块中*/
int SetBDeviceDataChannel(int DeviceType,int StationNum)
{
    if (DeviceType == 2){
          switch (StationNum) {
           case 1 ... 8:
           BDevice8StationSelectedNum =1;
               break;
           case 9 ... 16:
           BDevice8StationSelectedNum =2;
               break;
           case 17 ... 24:
           BDevice8StationSelectedNum =3;
               break;
           case 25 ... 32:
           BDevice8StationSelectedNum =4;
               break;
           case 33 ... 40:
           BDevice8StationSelectedNum =5;
               break;
           case 41 ... 48:
           BDevice8StationSelectedNum =6;
               break;
           case 49 ... 56:
           BDevice8StationSelectedNum =7;
               break;
           case 57 ... 64:
           BDevice8StationSelectedNum =8;
               break;

           default:
               break;
            }
            return RT_EOK;
    }else {
            return RT_ERROR;
        }
}

/*依据C设备的类型及站编号，算出站位于哪个八站模块中*/
int SetCDeviceDataChannel(int DeviceType,int StationNum)
{
    if (DeviceType == 3){
          switch (StationNum) {
           case 1 ... 8:
           CDevice8StationSelectedNum =1;
               break;
           case 9 ... 16:
           CDevice8StationSelectedNum =2;
               break;
           case 17 ... 24:
           CDevice8StationSelectedNum =3;
               break;
           case 25 ... 32:
           CDevice8StationSelectedNum =4;
               break;
           case 33 ... 40:
           CDevice8StationSelectedNum =5;
               break;
           case 41 ... 48:
           CDevice8StationSelectedNum =6;
               break;

           default:
               break;
            }
            return RT_EOK;
    }else {
            return RT_ERROR;
        }
}

/*检测是否已经有站点打开；如果没有，则打开任意一个站点前先打开公共模块；如果没有打开的，返回0，如果有已经打开的，返回1；用于打开站点的时候检测*/
int IsAnyStationON(int DeviceType)
{
    int status = 0X00;
    if (DeviceType ==1) {
        for (int var = 0;  var < 8;  var++) {
            status += ADeviceStatus[var] | 0X00 ;
        }
        if (status ==0)
        {
            return 0;
        }else{
            return 1;
        }
    } else if (DeviceType ==2) {
        for ( int var = 0; var < 8;  var++) {
            status += BDeviceStatus[var] | 0X00 ;
        }
        if (status ==0)
        {
            return 0;
        }else{
            return 1;
        }
    } else if (DeviceType ==3) {
        for ( int var = 0; var < 6;  var++) {
            status += CDeviceStatus[var] | 0X00 ;
        }
        if (status ==0)
        {
            return 0;
        }else{
            return 1;
        }
    }
}

/*检测是否所有的站点都已经关闭，如果都已经关闭，则最后关闭公共模块；如果都关闭了，则返回0，如果还有没有关闭的，则返回1；用于关闭站点的时候检测*/
int IsAllAStationOFF()
{
    int status = 0X00;
    int ret =0;
       for (int var = 0;  var < 8;  var++) {
           status += ADeviceStatus[var] & 0XFF ;
           if (status !=0)
                      {
                          ret =1;
                          break;
                      }
       }
       return ret ;
}
int IsAllBStationOFF()
{
    int status = 0X00;
    int ret =0;
       for (int var = 0;  var < 8;  var++) {
         status += BDeviceStatus[var] & 0XFF ;
         if (status !=0)
                    {
                        ret =1;
                        break;
                    }
     }
       return ret ;
}
int IsAllCStationOFF()
{
    int status = 0X00;
    int ret =0;
       for (int var = 0;  var < 6;  var++) {
         status += CDeviceStatus[var] & 0XFF ;
         if (status !=0)
                    {
                        ret =1;
                        break;
                    }
     }
     return ret ;
}

//向相应的站点发送设置的数据，并进行锁存设定;动作有设定为ON（1）和设定为OFF（2）两种。
//3.设定某一个站的时候，其他站的状态不能改变，所以需要有个变量储存当前所有八站模块中，每个站点的现在状态，这样才能做到只改变其中一位的数据。改变成功了一个需要记录下来当前的状态
//如果是ON操作，则开始操作以前，检测一下IsAnyStationON，用于判断是否打开公共模块；如果是OFF操作，则在操作完成以后，最后检测一下IsAllAStationOFF，如果都关闭了则最后关闭公共模块
//1.如果没有站点打开，则先开公共站，后打开每个站点；2.如果所有的站点都关闭，则最后关闭公共站。
//首先根据StationNum的数据判断出位于哪个八站模块，取出对应的status数据，然后依据opetation对应的位进行操作，并更新status的数据
//四个信号分别是：ABOUTPUTENABLEOUTPUT  ABSTBOUTPUT  ABSDATAOUTPUT  ABCLOCKOUTPUT；AP111IN-配置为中断模式。
int SendADeviceData(int StationNum,int Operation)
{
    int BitMask = 0x80;
    ADevice8StationSelectedNum =0;//每次进入这个函数，需要将这里重新置为0，这样能正常的设定status数组中的值
    /*使能新主板的数据通讯*/
    rt_pin_write(ADeviceSelSet, PIN_HIGH);
    rt_pin_write(AB4245ENABLEOUTPUT, PIN_HIGH);
    /*判断及对公共模块的操作*/
    if (Operation ==1) {
        if(IsAnyStationON(1)==0){ // 没有已经打开的
            LOG_D("No Stations OPEN!");//没有已经打开的
            //在打开站点之前打开公共模块
            rt_pin_write(AComModelEnable, PIN_HIGH);
        }
    }
    /*对相应的站点发送对应的操作的数据；先取出对应八站的数据，然后进行为操作，再八路信号同时发送出去，并更新status，其中三路控制信号已经在SetDeviceTypePins中设置完毕*/
    if(SetADeviceDataChannel(1,StationNum) == RT_EOK)
    {
        if (Operation ==1) {
            ADeviceStatusTemp[ADevice8StationSelectedNum-1] = (ADeviceStatus[ADevice8StationSelectedNum-1])|(0x01<<(StationNum-(ADevice8StationSelectedNum-1)*8-1));
         //   LOG_D("Operation ==1,ADeviceStatusTemp = %d",ADeviceStatusTemp);
         //   LOG_D("Operation ==1,ADevice8StationSelectedNum = %d",ADevice8StationSelectedNum);
        } else if (Operation ==2) {

       //    LOG_D("Operation ==2,!(1<<(StationNum-(ADevice8StationSelectedNum-1)*8-1)) = %d",~(0x01<<(StationNum-(ADevice8StationSelectedNum-1)*8-1)));
            ADeviceStatusTemp[ADevice8StationSelectedNum-1] = (ADeviceStatus[ADevice8StationSelectedNum-1])&(~(0x01<<(StationNum-(ADevice8StationSelectedNum-1)*8-1)));
       //     LOG_D("Operation ==2,ADeviceStatusTemp = %d",ADeviceStatusTemp[ADevice8StationSelectedNum-1]);
       //     LOG_D("Operation ==2,ADevice8StationSelectedNum = %d",ADevice8StationSelectedNum);
        }
        ADeviceStatus[ADevice8StationSelectedNum-1] = ADeviceStatusTemp[ADevice8StationSelectedNum-1];
        for (int var = 0; var < 8; var++) {
            //LOG_D("ADeviceStatus[var]=%d",ADeviceStatus[var]);
            ADeviceStatusTemp[var] = ADeviceStatus[var];
        }
       //下面是开始发送八位数据
         //首先将所有的数据位拉低
        for (int var = 0; var < 8; var++) {
            rt_pin_write(ADevicePins[var], PIN_LOW);
        }

        rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
        rt_hw_us_delay(100);
        rt_pin_write(ABSTBOUTPUT, PIN_LOW);
        rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_LOW);

        for (int var = 0; var < 8; var++) {    //传输八位数据，其中高位的数据先传输出去
            rt_hw_us_delay(2);

           ((ADeviceStatusTemp[0] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[0], PIN_HIGH) : rt_pin_write(ADevicePins[0], PIN_LOW);
           ((ADeviceStatusTemp[1] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[1], PIN_HIGH) : rt_pin_write(ADevicePins[1], PIN_LOW);
           ((ADeviceStatusTemp[2] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[2], PIN_HIGH) : rt_pin_write(ADevicePins[2], PIN_LOW);
           ((ADeviceStatusTemp[3] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[3], PIN_HIGH) : rt_pin_write(ADevicePins[3], PIN_LOW);
           ((ADeviceStatusTemp[4] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[4], PIN_HIGH) : rt_pin_write(ADevicePins[4], PIN_LOW);
           ((ADeviceStatusTemp[5] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[5], PIN_HIGH) : rt_pin_write(ADevicePins[5], PIN_LOW);
           ((ADeviceStatusTemp[6] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[6], PIN_HIGH) : rt_pin_write(ADevicePins[6], PIN_LOW);
           ((ADeviceStatusTemp[7] & BitMask)>> 7 == 1) ? rt_pin_write(ADevicePins[7], PIN_HIGH) : rt_pin_write(ADevicePins[7], PIN_LOW);

            //rt_hw_us_delay(5);
            rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
          //rt_hw_us_delay(30);
            rt_hw_us_delay(200);
            rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
            rt_hw_us_delay(200);
          //rt_pin_write(ABSDATAOUTPUT, PIN_LOW);
            for (int var = 0; var < 8; var++) {
                rt_pin_write(ADevicePins[var], PIN_LOW);
            }
            for (int var = 0; var < 8; var++) {
                ADeviceStatusTemp[var] <<= 1;
            }
        }
            /*输出数据使能，使其动作*/
           rt_hw_us_delay(50);
           rt_pin_write(ABSTBOUTPUT, PIN_HIGH); //先让STB动作，再让enable动作
           rt_hw_us_delay(50);//这个地方特别注意，因为时间短了在四个八站模块的时候没有问题，但加上八个八站模块以后就有问题。
           rt_pin_write(ABSTBOUTPUT, PIN_LOW);
           rt_hw_us_delay(80);

           rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_HIGH);
           rt_hw_us_delay(50);
            //rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_LOW);
           /*8位数据传输完成后，最后有一个长高位*/
           rt_hw_us_delay(100);
           rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
           rt_hw_us_delay(200);
           rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
          /*CLK的长高位了以后一段时间，再进行拉高*/
           rt_thread_mdelay(2);
           rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
           /*AP111IN接收到上升沿的中断的时候，证明数据传输完毕，后面会更新status中的数据*/

           /*此处需要将AllStationOFF设置为1，方便在alarm中判断是否所有的站点都已经关闭，方便关闭alarm*/
           AllStationOFF=1;
    }

    if (Operation ==2){
        if(IsAllAStationOFF()==0){  //所有的都关闭了
            LOG_D("ALL Stations closed!");//所有的都关闭了
            //在关闭站点以后关闭公共模块；
         //   rt_thread_mdelay(500);
            AllStationOFF = 0;
            rt_pin_write(AComModelEnable, PIN_LOW);

            //  失去使能新主板的数据通讯,这个不能在每个指令结束的时候失去使能，因为outputenable会变为低电平，这样的话后面的关闭指令就没法使继电器动作，
            //只能在最后所有的站点都关闭了以后，证明信主板不会继续操作设备了，才将新主板失去使能。控制权交给旧主板。
             rt_pin_write(ADeviceSelSet, PIN_LOW);
             rt_pin_write(AB4245ENABLEOUTPUT, PIN_LOW);
        }
    }
}

//向相应的站点发送设置的数据，并进行锁存设定;动作有设定为ON（1）和设定为OFF（2）两种。
//如果是ON操作，则开始操作以前，检测一下IsAnyStationON，用于判断是否打开公共模块；如果是OFF操作，则在操作完成以后，最后检测一下IsAllBStationOFF，如果都关闭了则关闭八站模块
//首先根据StationNum的数据判断出位于哪个八站模块，取出对应的status数据，然后依据opeton对应的位进行操作，并更新status的数据
//四个信号分别是：ABOUTPUTENABLEOUTPUT  ABSTBOUTPUT  ABSDATAOUTPUT  ABCLOCKOUTPUT；cpu64-配置为中断模式。
int SendBDeviceData(int StationNum,int Operation)
{
    int BitMask = 0x80;
    BDevice8StationSelectedNum =0;//每次进入这个函数，需要将这里重新置为0，这样能正常的设定status数组中的值
    /*使能新主板的数据通讯*/
    rt_pin_write(BDeviceSelSet, PIN_HIGH);
    rt_pin_write(AB4245ENABLEOUTPUT, PIN_HIGH);
    /*判断及对公共模块的操作*/
    if (Operation ==1) {
        if(IsAnyStationON(2)==0){ // 没有已经打开的
            LOG_D("NO Stations Opened!");//没有已经打开的

            //在打开站点之前打开公共模块
         rt_pin_write(AComModelEnable, PIN_HIGH);
        }
    }

    /*对相应的站点发送对应的操作的数据；先取出对应八站的数据，然后进行为操作，再发送出去，并更新status，其中三路控制信号已经在SetDeviceTypePins中设置完毕*/
    if(SetBDeviceDataChannel(2,StationNum) == RT_EOK)
    {
        if (Operation ==1) {
            BDeviceStatusTemp[BDevice8StationSelectedNum-1] = (BDeviceStatus[BDevice8StationSelectedNum-1])|(0x01<<(StationNum-(BDevice8StationSelectedNum-1)*8-1));
   //         LOG_D("Operation ==1,BDeviceStatusTemp = %d",BDeviceStatusTemp[BDevice8StationSelectedNum-1]);
        } else if (Operation ==2) {
            BDeviceStatusTemp[BDevice8StationSelectedNum-1] = (BDeviceStatus[BDevice8StationSelectedNum-1])&(~(0x01<<(StationNum-(BDevice8StationSelectedNum-1)*8-1)));
   //         LOG_D("Operation ==2,BDeviceStatusTemp = %d",BDeviceStatusTemp[BDevice8StationSelectedNum-1]);
        }
        BDeviceStatus[BDevice8StationSelectedNum-1] = BDeviceStatusTemp[BDevice8StationSelectedNum-1];
        for (int var = 0; var < 8; var++) {
   //         LOG_D("BDeviceStatus[var]=%d",BDeviceStatus[var]);
            BDeviceStatusTemp[var] = BDeviceStatus[var];
        }
       //下面是开始发送八位数据
        for (int var = 0; var < 8; var++) {
            rt_pin_write(BDevicePins[var], PIN_LOW);
        }
       // rt_pin_write(ABSDATAOUTPUT, PIN_LOW);
        rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
         //rt_hw_us_delay(100);
        rt_hw_us_delay(100);
        rt_pin_write(ABSTBOUTPUT, PIN_LOW);
        rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_HIGH);

        for (int var = 0; var < 8; var++) {    //传输八位数据，其中高位的数据先传输出去
            rt_hw_us_delay(2);

            ((BDeviceStatusTemp[0] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[0], PIN_HIGH) : rt_pin_write(BDevicePins[0], PIN_LOW);
            ((BDeviceStatusTemp[1] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[1], PIN_HIGH) : rt_pin_write(BDevicePins[1], PIN_LOW);
            ((BDeviceStatusTemp[2] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[2], PIN_HIGH) : rt_pin_write(BDevicePins[2], PIN_LOW);
            ((BDeviceStatusTemp[3] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[3], PIN_HIGH) : rt_pin_write(BDevicePins[3], PIN_LOW);
            ((BDeviceStatusTemp[4] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[4], PIN_HIGH) : rt_pin_write(BDevicePins[4], PIN_LOW);
            ((BDeviceStatusTemp[5] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[5], PIN_HIGH) : rt_pin_write(BDevicePins[5], PIN_LOW);
            ((BDeviceStatusTemp[6] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[6], PIN_HIGH) : rt_pin_write(BDevicePins[6], PIN_LOW);
            ((BDeviceStatusTemp[7] & BitMask)>> 7 == 1) ? rt_pin_write(BDevicePins[7], PIN_HIGH) : rt_pin_write(BDevicePins[7], PIN_LOW);

           // rt_hw_us_delay(5);
         //   rt_hw_us_delay(10);
            rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
          //  rt_hw_us_delay(30);
            rt_hw_us_delay(2);
            rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
            rt_hw_us_delay(2);
           // rt_pin_write(ABSDATAOUTPUT, PIN_LOW);
            for (int var1 = 0; var1 < 8; var1++) {
                      rt_pin_write(BDevicePins[var1], PIN_LOW);
                  }
            for (int var2 = 0; var2 < 8; var2++) {
                BDeviceStatusTemp[var2] <<= 1;
            }
        }

        /*输出数据使能，使其动作*/
           rt_hw_us_delay(50);
           rt_pin_write(ABSTBOUTPUT, PIN_HIGH); //先让STB动作，再让enable动作
           //rt_hw_us_delay(5);
           rt_hw_us_delay(20);//这个地方特别注意，因为时间短了在四个八站模块的时候没有问题，但加上八个八站模块以后就有问题。
           rt_pin_write(ABSTBOUTPUT, PIN_LOW);
              // rt_hw_us_delay(80);
           /*此处需要将AllStationOFF设置为1，方便在alarm中判断是否所有的站点都已经关闭，方便关闭alarm*/
           AllStationOFF=1;

              // rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_HIGH);
           /*    rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_HIGH);
               rt_hw_us_delay(50);
               rt_pin_write(ABOUTPUTENABLEOUTPUT, PIN_LOW);*/

           /*    8位数据传输完成后，最后有一个长高位
               rt_hw_us_delay(100);
               rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);
               rt_hw_us_delay(200);
               rt_pin_write(ABCLOCKOUTPUT, PIN_LOW);
              CLK的长高位了以后一段时间，再进行拉高
               rt_thread_mdelay(2);
               rt_pin_write(ABCLOCKOUTPUT, PIN_HIGH);*/
               /*AP111IN接收到上升沿的中断的时候，证明数据传输完毕，后面会更新status中的数据*/
    }

    if (Operation ==2){
            if(IsAllBStationOFF()==0){  //所有的都关闭了
                LOG_D("ALL Stations Closed!");//所有的都关闭了
                //在关闭站点以后关闭公共模块
                AllStationOFF = 0;
                /*失去使能新主板的数据通讯*/
                rt_pin_write(BDeviceSelSet, PIN_LOW);
                rt_pin_write(AB4245ENABLEOUTPUT, PIN_LOW);
                   }
        }
}

int SendCDeviceData(int StationNum,int Operation)
{
    int BitMask = 0x01;
       CDevice8StationSelectedNum =0;//每次进入这个函数，需要将这里重新置为0，这样能正常的设定status数组中的值
       /*使能新主板的数据通讯*/
       rt_pin_write(C4245ENABLE, PIN_LOW);
        rt_pin_write(CDeviceSelOUTPUT, PIN_HIGH);
      //判断及对公共模块的操作
       if (Operation ==1) {
           if(IsAnyStationON(3)==0){ // 没有已经打开的
               LOG_D("NO Stations Opened!");//没有已经打开的!
               //在打开站点之前打开公共模块
           //    rt_pin_write(CComModelEnable, PIN_HIGH);
           }
       }
    //   对相应的站点发送对应的操作的数据；先取出对应八站的数据，然后进行位的操作，将数据变为期望状态，再发送出去执行操作，并更新status，其中三路控制信号已经在SetDeviceTypePins中设置完毕
           if(SetCDeviceDataChannel(3,StationNum) == RT_EOK)
           {
               if (Operation ==1) {
                   CDeviceStatusTemp[CDevice8StationSelectedNum-1] = (CDeviceStatus[CDevice8StationSelectedNum-1])|(0x01<<(StationNum-(CDevice8StationSelectedNum-1)*8-1));
                //   LOG_D("Operation ==1,CDeviceStatusTemp = %d",CDeviceStatusTemp[CDevice8StationSelectedNum-1]);
               } else if (Operation ==2) {
                   CDeviceStatusTemp[CDevice8StationSelectedNum-1] = (CDeviceStatus[CDevice8StationSelectedNum-1])&(~(0x01<<(StationNum-(CDevice8StationSelectedNum-1)*8-1)));
                //   LOG_D("Operation ==2,CDeviceStatusTemp = %d",CDeviceStatusTemp[CDevice8StationSelectedNum-1]);
               }
               CDeviceStatus[CDevice8StationSelectedNum-1] = CDeviceStatusTemp[CDevice8StationSelectedNum-1];
               for (int var = 0; var < 8; var++) {
              //     LOG_D("BDeviceStatus[var]=%d",CDeviceStatus[var]);
                   CDeviceStatusTemp[var] = CDeviceStatus[var];
               }
               /*设置clr引脚为高电平，各个八站可以正常接受数据并动作，如果是低电平则会复位*/
               rt_pin_write(C8STATIONCLROUTPUT,PIN_HIGH);
               /*设置74LVX4245的22脚enable为低电平，设定传输方向为B 到 A*/
               rt_pin_write(C4245ENABLE, PIN_LOW);
               /*在下面传输数据之前，将计数器数据清零，方便计数器从零开始*/
               rt_pin_mode(C74HC161MROUTPUT, PIN_MODE_OUTPUT);
               rt_pin_write(C74HC161MROUTPUT,PIN_LOW);
               rt_pin_write(C74HC161MROUTPUT,PIN_HIGH);

               /*再快速将所有的与主板相连的6个引脚都拉高，全部不选*/
               rt_pin_write(C138CLEAR, PIN_LOW);
              //下面是开始发送48位数据,6个八站，每个8位
               for (int var1 = 0; var1 < 6; var1++)
               {
                   for (int var = 0; var < 8; var++)
                   {
                        /*C8STATIONADDRESSOUTPUT的一个周期中高电平加低电平共10ms,这样的话总共80ms与旧主板相同*/
                        rt_pin_write(C8STATIONADDRESSOUTPUT,PIN_LOW);

                        ((CDeviceStatusTemp[var1] & BitMask) == 1) ? rt_pin_write(C8STATIONDATAOUTPUT, PIN_HIGH) : rt_pin_write(C8STATIONDATAOUTPUT, PIN_LOW);
                       // rt_thread_mdelay(3);
                        rt_hw_us_delay(500);
                        /*对应的引脚拉低,选中对应的站对应的与主板相连的引脚
                                                            在拉低之前将138的6号引脚（clear）拉低，将138的三个引脚设置完毕后在将clr拉高，以免下面对三个引脚操作的时候无法同时进行
                                                            导致出现多个引脚都有拉低的电平动作，会有误操作问题。设定完毕后再将6号置高，对应的拉低开始起作用*/
                        rt_pin_write(C138CLEAR, PIN_LOW);
                        rt_pin_write(C138A0OUTPUT, CDevicePins[(var1*3)]);
                        rt_pin_write(C138A1OUTPUT, CDevicePins[(var1*3)+1]);
                        rt_pin_write(C138A2OUTPUT, CDevicePins[(var1*3)+2]);
                        rt_pin_write(C138CLEAR, PIN_HIGH);
                        rt_hw_us_delay(3);

                        //再快速将所有的与主板相连的引脚都拉高，全部不选
                        rt_pin_write(C138CLEAR, PIN_LOW);
                        //无论那个站点的数据是0还是1，这里都将数据拉低
                        rt_pin_write(C8STATIONDATAOUTPUT, PIN_LOW);

                       // rt_thread_mdelay(1);
                        rt_hw_us_delay(10);
                        rt_pin_write(C8STATIONADDRESSOUTPUT,PIN_HIGH);
                        //rt_thread_mdelay(6);
                        rt_hw_us_delay(20);
                        rt_pin_write(C8STATIONADDRESSOUTPUT,PIN_LOW);
                        /*数据移位*/
                        CDeviceStatusTemp[var1] >>= 1;
                    }
             }
               /*此处需要将AllStationOFF设置为1，方便在alarm中判断是否所有的站点都已经关闭，方便关闭alarm*/
               AllStationOFF=1;
           }
           if (Operation ==2){
                  if(IsAllCStationOFF()==0){  //所有的都关闭了
                      LOG_D("ALL Stations Closed!");//所有的都关闭了!
                      AllStationOFF = 0;
                      //在关闭站点以后关闭公共模块；
                   //   rt_thread_mdelay(500);
              //        rt_pin_write(CComModelEnable, PIN_LOW);
                      /*使能新主板的数据通讯*/
                           rt_pin_write(C4245ENABLE, PIN_HIGH);
                            rt_pin_write(CDeviceSelOUTPUT, PIN_LOW);
                  }
              }

         /*  失去使能新主板的数据通讯
           rt_pin_write(C4245ENABLE, PIN_HIGH);
            rt_pin_write(CDeviceSelOUTPUT, PIN_LOW);*/


}

/*
 * *此函数为闹钟的回调函数，其中首先检查是不是所有的数据都为0，如果是的话，证明都不需要定时了，直接退出
 * *在闹钟的回调函数中，检查是否有数据为1，如果为1的数据，证明本次进入回调函数就需要关闭站点了，对相应的站点发送关闭的指令，并将数据置位0
 * * 对所有的不等于0的数据执行减一的操作，将对应站点的定时时间减去一分钟，方便下次进入站点的时候进行判断
**/
void user_alarm_callback(rt_alarm_t alarm, time_t timestamp)
{
   /* rt_kprintf("user alarm callback function++++++++++++++++++++++++++++++++++++++++++++.\n");
    for (int var1 = 0; var1 < Device8StationsNum2*8; var1++) {
             rt_kprintf("TimeToRemain[ %d] = %d！.\n",var1,TimeToRemain[var1]);
         }*/
    for (int var2 = 0; var2 < Device8StationsNum2*8; var2++) {

                        if (TimeToRemain[var2]> 1) {
                                TimeToRemain[var2] =TimeToRemain[var2]-1;
                                }else if (TimeToRemain[var2] ==1){
                                 //   rt_kprintf("TimeToRemain[var2] ==1！，开始关闭站点！.\n");
                                    switch (DeviceType) {
                                         case 1:
                                             SendADeviceData(var2+1,2);
                                             ret = RT_EOK;
                                             break;
                                         case 2:
                                             SendBDeviceData(var2+1,2);
                                             ret = RT_EOK;
                                             break;
                                         case 3:
                                             SendCDeviceData(var2+1,2);
                                             ret = RT_EOK;
                                             break;
                                         default:
                                             ret = RT_ERROR;
                                             break;
                                        }
                                       TimeToRemain[var2] =0;
                                    }
                      }

    if ( AllStationOFF==0){
     rt_alarm_stop(alarm);
       rt_kprintf("ALL station timeremain is 0 or All station is closed");//所有的定时都为0或者所有站点都关闭了了,,.\n
        return;
    }
    /*rt_kprintf("user alarm callback function++++++++++++++++++++++++++++++++++++++++++++.\n");
       int sum=0;
       for (int var1 = 0; var1 < Device8StationsNum*8; var1++) {
           rt_kprintf("TimeToRemain[var1] = %d！.\n",TimeToRemain[var1]);
           sum |= TimeToRemain[var1];
       }

       if ((sum == 0) || (IsAllStationOFF(DeviceType))==0){
        rt_alarm_stop(alarm);
          rt_kprintf("所有的定时都为0或者所有站点都关闭了了,,.\n");
           return;
       }else {
               for (int var2 = 0; var2 < Device8StationsNum*8; var2++) {

                       if (TimeToRemain[var2]> 0) {

                               if (TimeToRemain[var2] ==1) {
                                   rt_kprintf("TimeToRemain[var2] ==1！，开始关闭站点！.\n");
                                   switch (DeviceType) {
                                             case 1:
                                                 SendADeviceData(var2+1,2);
                                                 ret = RT_EOK;
                                                 break;
                                             case 2:
                                                 SendBDeviceData(var2+1,2);
                                                 ret = RT_EOK;
                                                 break;
                                             case 3:
                                                 SendCDeviceData(var2+1,2);
                                                 ret = RT_EOK;
                                                 break;
                                             default:
                                                 ret = RT_ERROR;
                                                 break;
                                   }
                                     TimeToRemain[var2] =0;
                               }else {
                                         TimeToRemain[var2] =TimeToRemain[var2]-1;
                                       }
                       }else {
                             等于0的站点，证明不需要进行关闭操作了
                       }
                     }
           }*/
}

/*
 * *打开闹钟，一分钟进入闹钟一次
 * */
void alarm_initandstart(void)
{
    struct rt_alarm_setup setup;

    static time_t now;
    struct tm p_tm;

    if (alarm != RT_NULL)
        return;

    /* 获取当前时间戳，并把下一秒时间设置为闹钟时间，因为最短为1分钟，最长为十几个小时，所以最短每一个分钟进去检查一下 */
    now = time(NULL) + 1;
    gmtime_r(&now,&p_tm);

    setup.flag = RT_ALARM_SECOND;//每分钟进入一次alarm,就是每个站点到时间了以后执行关闭动作RT_ALARM_MINUTE
    setup.wktime.tm_year = p_tm.tm_year;
    setup.wktime.tm_mon = p_tm.tm_mon;
    setup.wktime.tm_mday = p_tm.tm_mday;
    setup.wktime.tm_wday = p_tm.tm_wday;
    setup.wktime.tm_hour = p_tm.tm_hour;
    setup.wktime.tm_min = p_tm.tm_min;
    setup.wktime.tm_sec = p_tm.tm_sec;

    alarm = rt_alarm_create(user_alarm_callback, &setup);
    rt_kprintf("create alarm!----------------------------------------\n");//创建alarm！
    if(RT_NULL != alarm)
    {
       // rt_alarm_start(alarm);
        rt_kprintf("init alarm success!----------------------------------\n");//初始化alarm成功
    }
}

/* export msh cmd */
MSH_CMD_EXPORT(alarm_initandstart,alarm_initandstart);


/*依据不同的设备类型，向不同的设备发送指令,如果有定时关闭的需求，则还需要打开定时器，并设定定时的*/
int SetABCDeviceStationAction(int StationNum,int RunMinutes,int Operation)
{
    int ret;

    if (RunMinutes > 0) {

        /*说明有定时的时间，需要设置时间*/
    //    LOG_I("SetABCDeviceStationAction---StationNum IS %d,runtime is %d",StationNum,RunMinutes);
        TimeToRemain[StationNum-1] = (RunMinutes)*60;

    } else {
        /*说明无定时的时间，正常执行下面的操作*/
    }
    switch (DeviceType) {
          case 1:
              SendADeviceData(StationNum,Operation);
              ret = RT_EOK;
              break;
          case 2:
              SendBDeviceData(StationNum,Operation);
              ret = RT_EOK;
              break;
          case 3:
              SendCDeviceData(StationNum,Operation);
              ret = RT_EOK;
              break;
          default:
              ret = RT_ERROR;
              break;
    }
    return ret;
}

/*根据DetectDeviceType的设备类型，设定相应的类型的设备的需要设定的引脚模式;并且设定三路控制信号的通路*/
int SetDeviceTypePins(int DeviceType)
{
    int ret;
    switch (DeviceType) {
        case 1:
            LOG_D("A1!");
            SetABDevicePins();
            /*设定CONTROSELECT为高，则连通A设备的3路控制信号，STB,CLOCK及outputenable*/
            rt_pin_write(CONTROSELECT, PIN_HIGH);
            rt_pin_write(AComModelEnable, PIN_LOW);
            ret = RT_EOK;
            break;
        case 2:
            LOG_D("B1!");
            SetABDevicePins();
           /* 设定为高，则连通B设备的3路控制信号，STB,CLOCK及outputenable*/
            rt_pin_write(CONTROSELECT, PIN_LOW);
            ret = RT_EOK;
            break;
        case 3:
            LOG_D("C1!");
            SetCDevicePins();
          //  rt_pin_write(CComModelEnable, PIN_LOW);
            ret = RT_EOK;
            break;
        default:
            ret = RT_ERROR;
            break;
    }
    return ret;
}

/*使用串口接受数据，依据输入的数据解析出控制哪一路设备及对应的站点数和不同的操作*/
static void setcmd(int argc, int**argv)
{
    if (argc < 4)
    {
        rt_kprintf("Please input'setcmd <a|b|c> stationNum operation'\n");
        return;
    }

    if (!rt_strcmp(argv[1], "a"))
    {
        rt_kprintf("a Devcie!\n");
        rt_kprintf("argv[0] = %s!\n",(argv[0]));
        rt_kprintf("argv[1] = %s!\n",(argv[1]));
        rt_kprintf("argv[2] = %d!\n",atoi(argv[2]));
        rt_kprintf("argv[3] = %d!\n",atoi(argv[3]));

        SendADeviceData(atoi(argv[2]),atoi(argv[3]));
    }
    else if (!rt_strcmp(argv[1], "b"))
    {
        rt_kprintf("b Devcie!\n");
        rt_kprintf("argv[0] = %s!\n",(argv[0]));
        rt_kprintf("argv[1] = %s!\n",(argv[1]));
        rt_kprintf("argv[2] = %d!\n",atoi(argv[2]));
        rt_kprintf("argv[3] = %d!\n",atoi(argv[3]));

        SendBDeviceData(atoi(argv[2]),atoi(argv[3]));

    }
    else if (!rt_strcmp(argv[1], "c"))
    {
        rt_kprintf("c Devcie!\n");
        rt_kprintf("argv[0] = %s!\n",(argv[0]));
        rt_kprintf("argv[1] = %s!\n",(argv[1]));
        rt_kprintf("argv[2] = %d!\n",atoi(argv[2]));
        rt_kprintf("argv[3] = %d!\n",atoi(argv[3]));
        SendCDeviceData(atoi(argv[2]),atoi(argv[3]));
    }
    else
    {
        rt_kprintf("setcmd <a|b|c> stationNum operation '\n");
    }
}
MSH_CMD_EXPORT(setcmd, setcmd sample: setcmd <a|b|c> stationNum operation);

int at_client_send1(int argc, char**argv)
{
    at_response_t resp = RT_NULL;

    if (argc != 2)
    {
        LOG_E("at_cli_send [command]  - AT client send commands to AT server.");
        return -RT_ERROR;
    }

    /* 创建响应结构体，设置最大支持响应数据长度为 512 字节，响应数据行数无限制，超时时间为 5 秒 */
    resp = at_create_resp(512, 0, rt_tick_from_millisecond(5000));
    if (!resp)
    {
        LOG_E("No memory for response structure!");
        return -RT_ENOMEM;
    }

    /* 发送 AT 命令并接收 AT Server 响应数据，数据及信息存放在 resp 结构体中 */
    if (at_exec_cmd(resp, argv[1]) != RT_EOK)
    {
        LOG_E("AT client send commands failed, response error or timeout !");
        return -RT_ERROR;
    }

    /* 命令发送成功 */
    LOG_D("AT Client send commands to AT Server success!");

    /* 删除响应结构体 */
    at_delete_resp(resp);

    return RT_EOK;
}

MSH_CMD_EXPORT(at_client_send1, AT Client send commands to AT Server and get response data);

/*------------------------------------------------------------------------------*/
/*static void ec20_urc_recv_func(struct at_client *client, const char *data, rt_size_t size)
{
    int device_socket = 0;
    rt_int32_t timeout;
    rt_size_t bfsz = 0, temp_size = 0;
    char *recv_buf = RT_NULL, temp[8] = {0};
    struct at_socket *socket = RT_NULL;
    struct at_device *device = RT_NULL;
    char *client_name = client->device->parent.name;

    RT_ASSERT(data && size);

    device = at_device_get_by_name(AT_DEVICE_NAMETYPE_CLIENT, client_name);
    if (device == RT_NULL)
    {
        LOG_E("get device(%s) failed.", client_name);
        return;
    }

  //   get the current socket and receive buffer size by receive data
    sscanf(data, "+QIURC: \"recv\",%d,%d", &device_socket, (int *) &bfsz);
  //   set receive timeout by receive buffer length, not less than 10 ms
    timeout = bfsz > 10 ? bfsz : 10;

    if (device_socket < 0 || bfsz == 0)
    {
        return;
    }

    recv_buf = (char *) rt_calloc(1, bfsz);
    if (recv_buf == RT_NULL)
    {
        LOG_E("no memory for URC receive buffer(%d).", bfsz);
         read and clean the coming data
        while (temp_size < bfsz)
        {
            if (bfsz - temp_size > sizeof(temp))
            {
                at_client_obj_recv(client, temp, sizeof(temp), timeout);
            }
            else
            {
                at_client_obj_recv(client, temp, bfsz - temp_size, timeout);
            }
            temp_size += sizeof(temp);
        }
        return;
    }

  //   sync receive data
    if (at_client_obj_recv(client, recv_buf, bfsz, timeout) != bfsz)
    {
        LOG_E("%s device receive size(%d) data failed.", device->name, bfsz);
        rt_free(recv_buf);
        return;
    }

 //    get at socket object by device socket descriptor
    socket = &(device->sockets[device_socket]);

   //  notice the receive buffer and buffer size
    if (at_evt_cb_set[AT_SOCKET_EVT_RECV])
    {
        at_evt_cb_set[AT_SOCKET_EVT_RECV](socket, AT_SOCKET_EVT_RECV, recv_buf, bfsz);
    }
}

static void ec20_urc_pdpdeact_func(struct at_client *client, const char *data, rt_size_t size)
{
    int connectID = 0;

    RT_ASSERT(data && size);

    sscanf(data, "+QIURC: \"pdpdeact\",%d", &connectID);

    LOG_E("context (%d) is deactivated.", connectID);
}

static void ec20_urc_dnsqip_func(struct at_client *client, const char *data, rt_size_t size)
{
    int i = 0, j = 0;
    char recv_ip[16] = {0};
    int result, ip_count, dns_ttl;
    struct at_device *device = RT_NULL;
    struct at_device_ec20 *ec20 = RT_NULL;
    char *client_name = client->device->parent.name;

    RT_ASSERT(data && size);

    device = at_device_get_by_name(AT_DEVICE_NAMETYPE_CLIENT, client_name);
    if (device == RT_NULL)
    {
        LOG_E("get device(%s) failed.", client_name);
        return;
    }
    ec20 = (struct at_device_ec20 *) device->user_data;

    for (i = 0; i < size; i++)
    {
        if (*(data + i) == '.')
            j++;
    }
     There would be several dns result, we just pickup one
    if (j == 3)
    {
        sscanf(data, "+QIURC: \"dnsgip\",\"%[^\"]", recv_ip);
        recv_ip[15] = '\0';

         set ec20 information socket data
        if (ec20->socket_data == RT_NULL)
        {
            ec20->socket_data = rt_calloc(1, sizeof(recv_ip));
            if (ec20->socket_data == RT_NULL)
            {
                return;
            }
        }
        rt_memcpy(ec20->socket_data, recv_ip, sizeof(recv_ip));


        ec20_socket_event_send(device, EC20_EVENT_DOMAIN_OK);
    }
    else
    {
        sscanf(data, "+QIURC: \"dnsgip\",%d,%d,%d", &result, &ip_count, &dns_ttl);
        if (result)
        {
            at_tcp_ip_errcode_parse(result);
        }
    }
}*/

uint8_t cjson_handle_uniqid(cJSON *json)
{
    printf("[I/%s] uniqid\n",DBG_TAG);
    return RT_EOK;
}
static int ec20_urc_uniqid_1_func(int StationNum,int Minutes,int Operation)
{
    int ret;
    ret = SetABCDeviceStationAction(StationNum,Minutes,1);
    return ret;
}
static int ec20_urc_uniqid_2_func(int StationNum,int Minutes,int Operation)
{
    int ret;

    ret = SetABCDeviceStationAction(StationNum,Minutes,2);
    ret = RT_EOK;
//    LOG_I("ec20_urc_uniqid_2_func");
    return ret;
}
static int ec20_urc_uniqid_3_func(int EightStationsNum)
{
    /*将定时数组中所有的时间都清零*/
    for (int var1 = 0; var1 < 64; var1++) {
          TimeToRemain[var1]=0;
    }

    int ret;
       switch (DeviceType) {
             case 1:
                 for (int var = 0; var < 64; var++) {
                     SendADeviceData(var,2);
                }
                 ret = RT_EOK;
                 break;
             case 2:
                 for (int var = 0; var < 64; var++) {
                      SendBDeviceData(var,2);
                }
                 ret = RT_EOK;
                 break;
             case 3:
                 for (int var = 0; var < 48; var++) {
                       SendCDeviceData(var,2);
                 }
                 ret = RT_EOK;
                 break;
             default:
                 ret = RT_ERROR;
                 break;
       }
    LOG_I("ec20_urc_uniqid_3_func");
    return ret;
}
static int ec20_urc_uniqid_4_func(char *DeviceStatus[])
{
    int ret;
    ret = RT_EOK;
    LOG_I("ec20_urc_uniqid_4_func");
    return ret;
}

/*接收到的URC的数据的处理函数*/
static void ec20_urc_qmtstat_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_ASSERT(data && size);
    LOG_I("ec20_urc_qmtstat_func,means received QMTSTAT,the status of MQTT Changed!");//说明收到了QMTSTAT,MQTT的状态变化了！！！！！！！！！！！！！！！！！！！！！！！
    sscanf(data,"%*[^:]:%s",str);//取出+QMTRECV: 以后的内容
    LOG_I(" str----------------------- is %s",str);

    MQTTConnected = 1;

}
/*    tempresult的错误信息及代表的意义：
 *    tempresult = 1;  no nodes
 *    tempresult = 2;  uniqid =1 but no run_minutes
 * */
static void ec20_urc_qmtrurc_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_ASSERT(data && size);
    intqmtrurc_func = 1;
 //   LOG_I(" <<<intqmtrurc_func === 1");

    int len;
    int uniqid =0;
    /*每次进入指令的解析前，将以下两个清零，重新接收数组长度及数组内容*/
    int Nodesarray_size =0;
    int *ReceiveNodes;
    int tempresult = 0;

     //  LOG_E(" data is %s",data);
    //sscanf(str,"%[^,],%[^,],%[^,],%s",receivecliend_idx,receivemessageid,receivetopic,receivemessage);可以取出，但是receivemessage前面多了一个“
    sscanf(data,"%*[^:]:%s",str);//取出+QMTRECV: 以后的内容
//    LOG_I(" str----------------------- is %s",str); //str2 is 0,7044,"test1","123456"
    sscanf(str,"%[^,],%[^,],%[^,],%s",receivecliend_idx,receivemessageid,receivetopic,receivemessage);
//    LOG_I(" cliend_idx---------- is %s",receivecliend_idx);
    /*LOG_E(" cliend_idx---------- is %s",receivecliend_idx);
    LOG_E(" messageid ----------is %s",receivemessageid);*/
   /* LOG_E(" topic ----------is %s",receivetopic);
    LOG_E(" receivemessage---------- is %s",receivemessage);*/

    /*因为接收的的receivemessage中前后带有引号，下面开始去除前后的“*/
    strcpy(receivemessage,receivemessage+1);
    len = strlen(receivemessage);
    /*去除末尾的引号*/
    if (receivemessage[len-1] == '"') {
        receivemessage[len-1] = '\0';
    }

//    LOG_I(" receivemessage new ---------- is %s",receivemessage);
//    LOG_I("device_id ---------- is %s",device_id3);
    /*--------------------下面开始解析receivemessage中的json----------------------------*/
    cJSON *root;
    root = cJSON_Parse(receivemessage);
    if (RT_NULL == root) {
        LOG_E("get root faild !\n");
        return -1;
    }

    /*首先判断device_id是否相等，相等才进行下一步处理，不相等说明不是本设备的命令*/
    cJSON *device_id_js = cJSON_GetObjectItem(root, "device_id");
//    LOG_I("device_id_js---------- is %s",device_id_js->valuestring);
    if (RT_NULL == device_id_js)
    {
       LOG_E("no device_id_js !\n");
       cJSON_Delete(root); //  free接受到的整个云端下发消息的json，没有device_id的话也需要将这个释放，不然会有内存泄漏的问题。
       intqmtrurc_func = 0;//这里需要将intqmtrurc_func设置为0，因为云端一下会发很多设备的查询指令，如果不设置为0，则查询线程中总是认为没有结束本函数中的操作
       LOG_I(" intqmtrurc_func === 0>>>");
       return -1;
    }
    else if (strcmp(device_id_js->valuestring,device_id3)==0)
          {
              //  LOG_E("device_id_js->valuestring)==device_id");

                /*先取出operation_id，无论uniqid是多少，都会有operation_id，后面会的接收命令回执会使用这个*/
                cJSON *operation_id_js = cJSON_GetObjectItem(root, "operation_id");
                if (RT_NULL == operation_id_js)
                {
                    LOG_E("no operation_id !\n");
                    cJSON_Delete(root);
                    intqmtrurc_func = 0;
                    LOG_I(" intqmtrurc_func === 0>>>");
                    return -1;
                }
                else
                    {
                    //    LOG_I("operation_id is %s",operation_id_js->valuestring);
                        strcpy( receivoperationid,operation_id_js->valuestring);
                   //     cJSON_Delete(operation_id_js);//这里需要释放，不然不知道是不是会引起串口占用始终增长v1.0.5中出现
                     //   LOG_E("operation_id is %s",receivoperationid);
                    }

                /*再判断uniqid，后面会对命令进行分开解析，不同的命令执行不同的操作*/
                 cJSON *uniqid_js = cJSON_GetObjectItem(root, "uniqid");
                if (RT_NULL == uniqid_js)
                {
                    LOG_E("no uniqid !\n");
                    cJSON_Delete(root);
                    intqmtrurc_func = 0;
                    LOG_I(" intqmtrurc_func === 0>>>");
                    return -1;
                }
                else if (((uniqid_js->valueint)==1)|((uniqid_js->valueint)==2) )//在uniqid=1或者是2的时候才会有数组nodes,才需要取出nodes;原来为uniqid：“1”的时候(atoi(uniqid_js->valuestring)==1)
                        {
                            uniqid = (uniqid_js->valueint);
                            LOG_I("uniqid =>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%d",uniqid);
                            /*uniqid=1或者2，后面会对命令进行分开解析，不同的命令执行不同的操作*/
                            cJSON *nodes_js = cJSON_GetObjectItem(root, "nodes");
                            if (RT_NULL == nodes_js)
                                {
                                    LOG_E("no nodes !\n");
                                    tempresult = 1;
                                }
                                else {/*有nodes数据则取出来*/
                                        /*取出或者打印出nodes数组中的每个元素的值*/
                                        Nodesarray_size =cJSON_GetArraySize(nodes_js);
                                        ReceiveNodes = (int *)malloc((Nodesarray_size)*4);

                                        int i =0;
                                        cJSON *item;
                                        char *p =NULL;
                                        for (int var = 0; var < Nodesarray_size; var++)
                                        {
                                           // item =cJSON_GetArrayItem(nodes_js, var);
                                            p = cJSON_PrintUnformatted(cJSON_GetArrayItem(nodes_js, var));
                                            *(ReceiveNodes+var) = atoi(p);
                                            free(p);
                                             //  LOG_I("nodes_js item is %d\n",*(ReceiveNodes+var));
                                           /* p = cJSON_PrintUnformatted(item);
                                            if(p){
                                                //   LOG_I("nodes_js item is %s\n",p);
                                                   *(ReceiveNodes+var) = atoi(p);
                                                //   LOG_I("ReceiveNodes[var] is %d\n",*(ReceiveNodes+var));
                                                   free(p);
                                                 }*/
                                        }
                                       // cJSON_Delete(nodes_js);//这里需要释放，不然不知道是不是会引起串口占用始终增长v1.0.5中出现
                                    }

                            /*如果是uniqid=1，还需要取出运行时间*/
                            if (((uniqid_js->valueint)==1))
                            {
                                cJSON *receivruntime_js = cJSON_GetObjectItem(root, "run_minutes");
                               if (RT_NULL == receivruntime_js)
                               {
                                   LOG_E("no receivruntime_js !\n");
                                   tempresult = 2;
                                  // return -1;
                               }
                               else
                                   {
                                      // strcpy( receivruntime,receivruntime_js->valueint);
                                       receivruntime=receivruntime_js->valueint;
                                       LOG_I("receivruntime_js is %d",receivruntime);
                                   }
                            }

                        }else if (((uniqid_js->valueint)==3)|((uniqid_js->valueint)==4))
                                {
                                      //  LOG_I("uniqid_js->valueint==3 |4,===%d",uniqid_js->valueint);
                                         /*取出uniqid的值，方便后面Switch使用*/
                                         uniqid = (uniqid_js->valueint);
                                         LOG_I("uniqid =>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%d",uniqid);

                                } else {
                                           LOG_E("uniqid is the wrong number !\n");
                                          // tempresult = 3;
                                      //     cJSON_Delete(uniqid_js);//这里需要释放，不然不知道是不是会引起串口占用始终增长v1.0.5中出现
                                           cJSON_Delete(root);
                                           intqmtrurc_func = 0;
                                           LOG_I(" intqmtrurc_func === 0>>>");
                                           return -1;
                                }
                       //  cJSON_Delete(uniqid_js);//这里需要释放，不然不知道是不是会引起串口占用始终增长v1.0.5中出现

                     /*创建回复消息的json数据-接收到指令的回执*/
                     cJSON *rootack;
                     rootack = cJSON_CreateObject();
                     if (RT_NULL == rootack)
                         {
                             LOG_E("get rootack faild !\n");
                             return -1;
                         }else {
                                 //  cJSON * js_body;
                                 //   cJSON_AddStringToObject(rootack, "uniqid", uniqid);
                                 cJSON_AddStringToObject(rootack, "operation_id", receivoperationid);
                                 cJSON_AddStringToObject(rootack, "device_id", device_id3);
                                 /*已经走到这里，证明指令中device_id，operation_id，uniqid，依次都是对的，但是有可能缺少nodes或者是runtine,
                                                                                    * 解析命令并回复，回复“指令收到”中result及msg根据tempresult的数据填入不同的内容*/
                                 if (tempresult == 0) {
                                     cJSON_AddNumberToObject(rootack, "result",0);

                                } else if (tempresult == 1) {
                                    cJSON_AddNumberToObject(rootack, "result",2);
                                    cJSON_AddStringToObject(rootack, "msg", "no nodes");

                                }else if(tempresult == 2) {
                                    cJSON_AddNumberToObject(rootack, "result",2);
                                    cJSON_AddStringToObject(rootack, "msg", "uniqid =1 but no run_minutes");
                                }


                                 char *s = cJSON_PrintUnformatted(rootack);
                                 //    char *s =cJSON_Print(rootack);
                                 /*接收到指令后，将接收到指令成功的回执返送回去*/
                                 PublishMessage(s);//将接收到指令的回执发送出去
                                         if(s){
                                                 LOG_I("ack json is %s\n",s);
                                                 free(s);
                                              }
                               }
                       //  cJSON_Delete(device_id_js);//v1.0.5
                         cJSON_Delete(rootack); //  free释放掉回复收到指令消息的json

          }else //命令中的device_id不相等的话直接丢弃，不返回任何消息
             {

         //          LOG_E("device_id_js->valuestring!=device_id3");
                 //  cJSON_Delete(device_id_js);//v1.0.5
                   cJSON_Delete(root); //  free接受到的整个云端下发消息的json，不是自己的指令也需要将这个释放，不然会有内存泄漏的问题。
                   intqmtrurc_func = 0;//这里需要将intqmtrurc_func设置为0，因为云端一下会发很多设备的查询指令，如果不设置为0，则查询线程中总是认为没有结束本函数中的操作
           //        LOG_I(" intqmtrurc_func === 0>>>");
                   return -1;
             }


    /*tempresult == 0的时候才会执行后续操作并回复指令执行成功的回执，如果不等于0，不会回复执行成功，只会执行到上述的收到指令成功回执为止，如果是tempresult=1或2还会返回msg*/
    if (tempresult == 0) {

            /*创建回复消息的json数据-指令执行完成的回执*/
             cJSON *rootackok;
             rootackok = cJSON_CreateObject();
             if (RT_NULL == rootackok)
                 {
                     LOG_E("get rootackok faild !\n");
                     return -1;
                 }else {
                        //  cJSON * js_body;
                        cJSON_AddStringToObject(rootackok, "operation_id", receivoperationid);
                        cJSON_AddStringToObject(rootackok, "device_id", device_id3);
                        }
                         /*这里 不能 将接收到指令成功的回执返送回去及释放掉，因为下面会细分并执行*/
                         //cJSON_Delete(rootackok);

             /*下面根据uniqid的不同，执行不同的操作，并回复不同的信息*/
             switch((uniqid))
                {
                case 1 :
                        LOG_E(" case 1");
                        ret=0;
                        retsum =0;

                        //取出站点数，动作类型及持续的时间传到下面函数
                        for (int var = 0; var < Nodesarray_size; var++) {
                            ret = ec20_urc_uniqid_1_func(*(ReceiveNodes+var),(receivruntime),1);
                            retsum += ret;
                        }
                        free(ReceiveNodes);

                        alarm_initandstart();
                        LOG_E("case 1 :-----------------------open alarm！！！");
                        rt_alarm_start(alarm);

                        /*依据指令执行的成功及失败状态，返回指令执行完成回执*/
                        if (retsum ==0) {
                            cJSON_AddNumberToObject(rootackok, "result",1);

                        } else {
                            cJSON_AddNumberToObject(rootackok, "result",2);
                            cJSON_AddStringToObject(rootackok, "msg", "the uniqid=1 command is not actived");
                        }

                        char *s1 = cJSON_PrintUnformatted(rootackok);
                         PublishMessage(s1);//将接收到指令的回执发送出去
                         if(s1){
                               //  LOG_I("ackOK json is %s\n",s1);
                                 free(s1);
                              }
                        cJSON_Delete(rootackok); //  free回复收到指令消息的json

                        LOG_I(" ec20_urc_1,open station");

                        break;
                case 2 :

                        LOG_E(" case 2");
                        ret=0;
                        retsum =0;
                        for (int var = 0; var < Nodesarray_size; var++) {
                            ret = ec20_urc_uniqid_2_func(*(ReceiveNodes+var),0,2);
                            retsum += ret;
                        }
                        free(ReceiveNodes);
                        /*依据指令执行的成功及失败状态，返回指令执行完成回执*/
                        if (retsum ==0) {
                             cJSON_AddNumberToObject(rootackok, "result",1);

                        } else {
                             cJSON_AddNumberToObject(rootackok, "result",2);
                             cJSON_AddStringToObject(rootackok, "msg", "the uniqid=2 command is not actived");
                        }

                        char *s2 = cJSON_PrintUnformatted(rootackok);
                         PublishMessage(s2);//将接收到指令的回执发送出去
                         if(s2){
                                 LOG_I("ackOK json is %s\n",s2);
                                 free(s2);
                              }
                        cJSON_Delete(rootackok); //  free回复收到指令消息的json
                        LOG_E(" ec20_urc_2，close station");

                        break;

                case 3 :

                       LOG_E(" case 3");

                    //   itoa(DeviceType,type,10);
                    //   itoa(Device8StationsNum,num,10);
                    //   LOG_E(" type is %s",type);
                    //   LOG_E(" num is %s",num);
                       //依据指令执行的成功及失败状态，返回指令执行完成回执
                       ret = ec20_urc_uniqid_3_func(1);

                       if (retsum ==0) {
                             cJSON_AddNumberToObject(rootackok, "result",1);
                       } else {
                             cJSON_AddNumberToObject(rootackok, "result",2);
                             cJSON_AddStringToObject(rootackok, "msg", "the uniqid=3 command is not actived");
                       }

                       char *s3 = cJSON_PrintUnformatted(rootackok);
                        PublishMessage(s3);//将接收到指令的回执发送出去
                        if(s3){
                                LOG_I("ackOK json is %s\n",s3);
                                free(s3);
                             }
                        cJSON_Delete(rootackok); //  free回复收到指令消息的json

                        LOG_E(" ec20_urc_3，close all stations");
                        break;

                case 4 :

                        LOG_E(" case 4");

                        ret=0;
                        retsum =0;
                      //  itoa(DeviceType,type,10);
                      //  itoa(Device8StationsNum,num,10);
                     //   LOG_E(" type is %s",type);
                     //   LOG_E(" num is %s",num);

                        /*如果没有安装八站模块，则返回错误*/
                      if (Device8StationsNum ==0) {
                            retsum=1;
                        }

                       /* 读取温度计湿度值*/
                        dht_device_t sensor = dht_create(DATA_PIN);
                        if(dht_read(sensor)) {

                            rt_int32_t temp = dht_get_temperature(sensor);
                            rt_int32_t humi = dht_get_humidity(sensor);
                            rt_kprintf("Temp: %d, Humi: %d\n", temp, humi);
                            temperature=temp;
                            humidity=humi;
                            retsum=0;
                        }
                        else {
                            rt_kprintf("Read dht sensor failed.\n");
                            retsum=1;
                        }
                        dht_delete(sensor);

                                            /*  读取功率的值*/
                        if (1) {
                            adc_vol_sample();
                            ia=(volvalue/100)/0.6;
                            retsum=0;
                        } else {
                            retsum=1;
                        }

                        /*读取信号的值*/
                        at_response_t resp;
                        resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
                         //  if (at_obj_exec_cmd((client), (resp), (cmd)) < 0)
                        at_obj_exec_cmd(at_client_get_first(), resp,"AT+CSQ");
                        at_resp_parse_line_args_by_kw(resp, "+CSQ:", "+CSQ: %d,%d", &signalstrength[0], &signalstrength[1]);
                        LOG_I("in  case 4 :signalstrength  number: %d",  signalstrength[0]);
                        at_delete_resp(resp);
                        /*依据指令执行的成功及失败状态，返回指令执行完成回执*/


                        if (retsum == 0) {
                              cJSON_AddNumberToObject(rootackok, "result",1);

                              if (DeviceType==1) {
                                  LOG_I("DeviceType==1\n");
                                  cJSON_AddStringToObject(rootackok, "type", "A");
                                  cJSON_AddNumberToObject(rootackok, "node_count",(Device8StationsNum2)*8);
                                  cJSON_AddItemToObject(rootackok, "node_status",cJSON_CreateIntArray(ADeviceStatus,8));

                               } else if (DeviceType==2){
                                          LOG_I("DeviceType==2\n");
                                          //      cJSON_AddItemToArray(js_body, cJSON_CreateIntArray(BDeviceStatus, 8));
                                          cJSON_AddStringToObject(rootackok, "type", "B");
                                          cJSON_AddNumberToObject(rootackok, "node_count",(Device8StationsNum2)*8);
                                          cJSON_AddItemToObject(rootackok, "node_status",cJSON_CreateIntArray(BDeviceStatus, 8));

                                        }else if (DeviceType==3) {
                                               LOG_I("DeviceType==3\n");
                                               cJSON_AddStringToObject(rootackok, "type", "C");
                                               cJSON_AddNumberToObject(rootackok, "node_count",(Device8StationsNum2)*8);
                                               cJSON_AddItemToObject(rootackok, "node_status",cJSON_CreateIntArray(CDeviceStatus, 6));
                                               }

                              cJSON_AddNumberToObject(rootackok, "i",ia);
                              cJSON_AddNumberToObject(rootackok, "u",12);
                               cJSON_AddNumberToObject(rootackok, "t",temperature/10.0);
                               cJSON_AddNumberToObject(rootackok, "h",humidity/10);
                              cJSON_AddNumberToObject(rootackok, "s",signalstrength[0]);
                        } else {
                                                                           /*指令执行失败*/
                            cJSON_AddNumberToObject(rootackok, "result",2);
                            cJSON_AddStringToObject(rootackok, "msg", "the uniqid=4 command is not actived");
                        }

                        char *s4 = cJSON_PrintUnformatted(rootackok);
                              PublishMessage(s4);//将接收到指令的回执发送出去
                              if(s4){
                                      LOG_I("ackOK json is %s\n",s4);
                                      free(s4);
                                   }
                             cJSON_Delete(rootackok); //  free回复收到指令消息的json
                            LOG_E(" ec20_urc_4，get all stations status");//获取所有站点的状态

                            break;

                default :

                        LOG_E(" default ec20_urc_qiurc_func.");
                        break;
                }
    }else {
        //什么都不做，不回复   执行执行成功   回执
        }

       cJSON_Delete(root); //  free接受到的整个云端下发消息的json，在所有的处理都完成了以后再释放整个
       intqmtrurc_func = 0;
       LOG_I(" intqmtrurc_func === 0>>>");

/*------------------------上面开始解析receivemessage中的json----------------------------*/

}

static const struct at_urc urc_table[] =
{
  //  {"SEND OK",     "\r\n",                 urc_send_func},
 //   {"SEND FAIL",   "\r\n",                 urc_send_func},
  //  {"+QIOPEN:",    "\r\n",                 urc_connect_func},
      {"+QMTSTAT:",     "\r\n",               ec20_urc_qmtstat_func},
      {"+QMTRECV:",     "\r\n",               ec20_urc_qmtrurc_func},
};


/*查询设备中所安装的SIM卡的卡号，作为deviceID，如果没有监测到卡号则报警*/
int GetDeviceID()
{
    int ret = 0;
    at_response_t resp;
    resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
    /*下面是查询4G电话卡中的号码（就是正常的手机号）,如果选用这个作为ID号码也可以*/
   /* send "AT+CNUM" commond to get simcard number*/
 /*  if (at_exec_cmd(resp, "AT+CNUM") < 0)
   {
       ret = -RT_ERROR;
   }
   if (at_resp_parse_line_args(resp,2,"%*[^\"]\"%[^\"]",device_id) <= 0)
   {
       LOG_E(" device prase \"AT+CNUM\" cmd error.");
       ret = -RT_ERROR;
   }
   LOG_D("device_ID number: %s",  device_id);
   at_delete_resp(resp);*/

    /* 下面是查询4G物联网卡中的ICCID作为设备ID,send "AT+QCCID commond to get simcard THE ICCID number*/
   if (at_exec_cmd(resp, "AT+QCCID") < 0)
     {
         ret = -RT_ERROR;
     }
     if (at_resp_parse_line_args_by_kw(resp,"+QCCID:","+QCCID: %s",device_id) <= 0)
     {
         LOG_E(" device prase \"AT+QCCID\" cmd error.");
         ret = -RT_ERROR;
     }
     LOG_D("device_ID number: %s",  device_id);
     at_delete_resp(resp);


   resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
   //  if (at_obj_exec_cmd((client), (resp), (cmd)) < 0)
   at_obj_exec_cmd(at_client_get_first(), resp,"AT+CSQ");
   at_resp_parse_line_args_by_kw(resp, "+CSQ:", "+CSQ: %d,%d", &signalstrength[0], &signalstrength[1]);
   LOG_I("signalstrength number: %d",  signalstrength[0]);


    /* resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
     if (at_exec_cmd(resp, "AT+CSQ") < 0)
         {
            ret = -RT_ERROR;
         }
     if (at_resp_parse_line_args(resp,2,"%*[^ ] %d,%d", signalstrength,biterrorrate)<=0) {
         LOG_E(" +CSQ: cmd error.");
         ret = -RT_ERROR;
     }
     LOG_I("signalstrength=%d,biterrorrate=%d", signalstrength,biterrorrate);
     at_delete_resp(resp);*/
   return ret;
   /*下面是查询imei号码,如果选用这个作为ID号码也可以*/
  /* resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
      send "AT+GSN" commond to get device IMEI
            if (at_exec_cmd(resp, "AT+GSN") < 0)
            {
                ret = -RT_ERROR;
               // goto __exit;
            }

            if (at_resp_parse_line_args(resp, 2, "%s", imei) <= 0)
            {
                LOG_E(" device prase \"AT+GSN\" cmd error.");
                ret = -RT_ERROR;
              //  goto __exit;
            }

            LOG_D("device IMEI number: %s",  imei);
            at_delete_resp(resp);*/
}

/*打开MQTT端口为后面连接准备*/
int OpenConnect(char* hostname,char* portnumber,char* clientid,char* username,char* password )
{
    int ret = 0;
    char registered[1] = {0};

    at_response_t resp;
    resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));

    /* AT+QMTOPEN=0,"post-cn-i7m2fwizj12.mqtt.aliyuncs.com",1883 */
    char *cmd = (char *)malloc(strlen(QMTOPENcmd)+strlen(equals)+strlen(cliend_idx)+strlen(",")
                              +strlen(host_name)+strlen(",")+strlen(port));
    sprintf(cmd, "%s=%s,%s,%s", QMTOPENcmd,cliend_idx,host_name,port);
    LOG_E("cmd: %s",  cmd);
    if (at_exec_cmd(resp, cmd) < 0)
    {
    LOG_E("AT+QMTOPEN err !");
    ret = -RT_ERROR;
    }
    free(cmd);
    at_delete_resp(resp);
    return ret;
}

/*通过MQTT连接到云端*/
int ConnectToCloud(char* hostname,char* portnumber,char* clientid,char* username,char* password )
{
    int ret = 0;
    char registered[1] = {0};

    at_response_t resp;
    resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));

    rt_thread_mdelay(100);
    if (at_exec_cmd(resp, "AT+QMTDISC=0") < 0)
       {
        LOG_E("AT+QMTDISC=0 !");
         //ret = -RT_ERROR;
       }
    at_delete_resp(resp);

    rt_thread_mdelay(200);

    resp = at_create_resp(256, 0, rt_tick_from_millisecond(5000));
    // GID_001

//     accessKey = ''

// #账号secre 从阿里云账号控制台获取
// secre = ''
   /* AT+QMTCONN=0,"GID_001@@@test_topic","Signature|LTAI5tJeLqp6F7DvqVxJiqpu|post-cn-i7m2fwizj12","WY2pdjSPiDl6Q3WkN6i6jP5y7W3zY1=" */
    
    
    /* AT+QMTCONN=0,"GID_test@@@FactoryOnIot","Signature|LTAI5t6E7XoRLUt4ewRupXA5|post-cn-i7m2fwizj12","gLtwTxFNsiqnGg2KJfWkSNGFTBI=" */

   char*cmd = (char *)malloc(strlen(QMTCONNcmd)+strlen(equals)+strlen(cliend_idx)+strlen(",")
                                 +strlen(clientid)+strlen(",")+strlen(username)+strlen(",")+strlen(password));
   sprintf(cmd, "%s=%s,%s,%s,%s", QMTCONNcmd,cliend_idx,clientid,username,password);
   LOG_E("cmd: %s",  cmd);
   rt_thread_mdelay(200);
   if (at_exec_cmd(resp, cmd) < 0)
   {
       LOG_E("AT+QMTCONN err !");
     ret = -RT_ERROR;
   }
   free(cmd);
   at_delete_resp(resp);
   return ret;

}

/*订阅相应的主题*/
int SubscribeToTopic(char *clientid,char * messageid,char *topic,char * qos)
{
    int ret = 0;
    at_response_t resp;
    resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
    /* AT+QMTSUB=0,1,"test1",0 */
    char *cmd = (char *)malloc(strlen(QMTSUBcmd)+strlen(equals)+strlen(cliend_idx)+strlen(",")
                              +strlen(messageid)+strlen(",")+strlen(recvtopic)+strlen(",")+strlen(qos));
    sprintf(cmd, "%s=%s,%s,%s,%s", QMTSUBcmd,cliend_idx,messageid,recvtopic,qos);
    LOG_E("cmd: %s",  cmd);
 //   rt_thread_mdelay(200);
    if (at_exec_cmd(resp, cmd) < 0)
    {
        LOG_E("AT+QMTSUB err !");
    ret = -RT_ERROR;
    }
    free(cmd);

    /* AT+QMTSUB=0,1,"test2",0
         *cmd = (char *)malloc(strlen(QMTSUBcmd)+strlen(equals)+strlen(cliend_idx)+strlen(",")
                                  +strlen(messageid)+strlen(",")+strlen(sendtopic)+strlen(",")+strlen(qos));
        sprintf(cmd, "%s=%s,%s,%s,%s", QMTSUBcmd,cliend_idx,messageid,sendtopic,qos);
        LOG_E("sendtopic cmd: %s",  cmd);
        rt_thread_mdelay(200);
        if (at_exec_cmd(resp, cmd) < 0)
        {
            LOG_E("AT+QMTSUB err !");
        ret = -RT_ERROR;
        }
        free(cmd);*/

     /*    AT+QMTuns=0,1,"test2"
                 *cmd = (char *)malloc(strlen(QMTUNScmd)+strlen(equals)+strlen(cliend_idx)+strlen(",")
                                          +strlen(messageid)+strlen(",")+strlen(sendtopic));
                sprintf(cmd, "%s=%s,%s,%s", QMTUNScmd,cliend_idx,messageid,sendtopic);
                LOG_E("UNStopic cmd: %s",  cmd);
                rt_thread_mdelay(200);
                if (at_exec_cmd(resp, cmd) < 0)
                {
                    LOG_E("AT+QMTUNS err !");
                ret = -RT_ERROR;
                }
                free(cmd);*/

    at_delete_resp(resp);
    return ret;
}

/*使EC20模块重启并重新连接到相应的接口并订阅相应的主题*/
int ReConnectAndSubscribe()
{
      int status1=0;
      int status2=0;
      int status3=0;
    /*将EC20模块进行复位，注意延时需要足够，不然后面的重连可能会失败*/
      rt_pin_write(GRESET, PIN_HIGH);
      rt_thread_mdelay(400);
      LOG_D("GRESET!");
      rt_pin_write(GRESET, PIN_LOW);

      rt_thread_mdelay(20000);


     status1=OpenConnect(host_name,port,clientid,Username,Password);
     LOG_E("2status1: %d",  status1);
     rt_thread_mdelay(100);
     status2=ConnectToCloud(host_name,port,clientid,Username,Password);
     LOG_E("2status2: %d",  status2);
     rt_thread_mdelay(500);
     status3=SubscribeToTopic(cliend_idx,messageid,recvtopic,qos);
     LOG_E("2status3: %d",  status3);
   //  status3=1;
     LOG_E("2----status3: %d",  status3);
     rt_thread_mdelay(100);

     if ((status1==0)&& (status2==0)&&(status3==0)){
         return 0;
      }else {
          return 1;
    }
}

/*向相应的主题发布上线消息*/
int PublishMessage( char *message)
{
    int ret = 0;
    itoa(strlen(message),messagelength,10);
 //   LOG_E("messagelength------------: %s",  messagelength);

    /*消息格式类似： AT+QMTPUBEX=0,1,1,0,"test1",5*/
       char *cmd = (char *)malloc(strlen(QMTPUBEXcmd)+strlen(equals)+strlen(cliend_idx)+strlen(",")
                                 +strlen(messageid)+strlen(",")+strlen(qos)+strlen(",")
                                 +strlen(retain)+strlen(",")+strlen(sendtopic)+strlen(",")+strlen(message));
       sprintf(cmd, "%s=%s,%s,%s,%s,%s,%s", QMTPUBEXcmd,cliend_idx,QOS0messageid,qos,retain,sendtopic,messagelength);
//       LOG_E("cmd: %s",  cmd);

    at_response_t resp;

    resp = at_create_resp(256, 0, rt_tick_from_millisecond(2000));//如果使用 at_obj_set_end_sign(at_client_get_first(), '>');
                                                                  //那这里的256后面的数据就是2，也就是第二行是“>”
    if (resp == RT_NULL)
       {
           LOG_E("no memory for resp create.");
           return -RT_ENOMEM;
       }

    /*因为此处将 AT+QMTPUBEX=。。。指令发出后，回复的是》，不是ok,所以此处不必等待，所以设定RT_NULL,直接下一步发送消息内容*/
       /* set AT client end sign to deal with '>' sign.*/
//       at_obj_set_end_sign(at_client_get_first(), '>');

  /*  if (at_obj_exec_cmd(at_client_get_first(),resp, cmd) == -2)
        {
        LOG_E("can not receive >--------------------------------------------------------------------------");
        ret = -RT_ERROR;
        }*/
    if (at_obj_exec_cmd(at_client_get_first(),RT_NULL, cmd) == -2)
        {
        LOG_E("can not receive >--------------------------------------------------------------------------");
        ret = -RT_ERROR;
        }

   /* if ((at_resp_parse_line_args_by_kw(resp, ">",">",&signalstrength[0]))==0) {

               LOG_E("can not receive >--------------------------------------------------------------------------");
               ret = -RT_ERROR;
    }*/

//    at_delete_resp(resp);
    /* 此处的延时是为了保证接收到了》*/
    rt_thread_mdelay(5);

    /* reset the end sign for data conflict */
//      at_obj_set_end_sign(at_client_get_first(), 0);
    //at_obj_set_end_sign("e0", 0);
    /* 下面是发送消息的部分*/

//    resp = at_create_resp(256, 0, rt_tick_from_millisecond(800));

    if (at_exec_cmd(resp, message) < 0)
    {
       ret = -RT_ERROR;
    }
    free(cmd);
    at_delete_resp(resp);
    return ret;
}

/*配置PVD的功能，使之可以在掉电的瞬间，将一些信息保存下来，这样的话再次上电的时候，可以继续执行未执行完成的指令*/
void PVD_Config(void)
{
  //  _HAL_RCC_PWR_CLK_ENABLE();

    /*##-2- Configure the NVIC for PVD #########################################*/
       HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
       HAL_NVIC_EnableIRQ(PVD_IRQn);

       /* Configure the PVD Level to 3 and generate an interrupt on rising and falling
             edges(PVD detection level set to 2.5V, refer to the electrical characteristics
             of you device datasheet for more details) */
          PWR_PVDTypeDef sConfigPVD;
          sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;
          sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
          HAL_PWR_ConfigPVD(&sConfigPVD);

          /* Enable the PVD Output */
          HAL_PWR_EnablePVD();
}

/*PVD的中断，在其中存取数据，方便上电的时候检测未执行完成的命令 */
void PVD_IRQHandler(void)
{
  /* USER CODE BEGIN PVD_IRQn 0 */

  /* USER CODE END PVD_IRQn 0 */
  HAL_PWR_PVD_IRQHandler();

  /* USER CODE BEGIN PVD_IRQn 1 */
  LOG_I("HAL_PWR_PVDCallback!");
  rt_pin_write(BEEPPIN, PIN_HIGH);
  /* USER CODE END PVD_IRQn 1 */
}

/**
  * @brief  PWR PVD interrupt callback
  * @retval None
  */
__weak void HAL_PWR_PVDCallback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PWR_PVDCallback could be implemented in the user file
*/
}

void PowerOnSendMessage()
{
    /*--------------------------------------------------------------------------*/
        /*上电启动的时候，需要上传4号命令，*/
        /*读取温度计湿度值*/
        dht_device_t sensor = dht_create(DATA_PIN);
        rt_thread_mdelay(5000);
        if(dht_read(sensor)) {

            rt_int32_t temp = dht_get_temperature(sensor);
            rt_int32_t humi = dht_get_humidity(sensor);
            rt_kprintf("Temp: %d, Humi: %d\n", temp, humi);
            temperature=temp;
            humidity=humi;
        }
        else {
            rt_kprintf("Read dht sensor failed.\n");
        }
        dht_delete(sensor);


        /*读取功率的值*/
        if (1) {
            adc_vol_sample();
            ia=(volvalue/100)/0.6;
            retsum=0;
        } else {
            retsum=1;
        }
        /*读取信号的值*/
        at_response_t resp;
        resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
         //  if (at_obj_exec_cmd((client), (resp), (cmd)) < 0)
        at_obj_exec_cmd(at_client_get_first(), resp,"AT+CSQ");
        at_resp_parse_line_args_by_kw(resp, "+CSQ:", "+CSQ: %d,%d", &signalstrength[0], &signalstrength[1]);
     //   LOG_I("in  case 4 :signalstrength  number: %d",  signalstrength[0]);

        /*创建上电后 上报消息 的json数据*/
         cJSON *onlineack;
         onlineack = cJSON_CreateObject();
         if (RT_NULL == onlineack)
             {
                 LOG_E("get onlineack faild !\n");
                 return -1;
             }else {
                    //  cJSON * js_body;
                    cJSON_AddStringToObject(onlineack, "operation_id", "0");
                    cJSON_AddStringToObject(onlineack, "device_id", device_id3);
                    }
        cJSON  *js_body;
        cJSON_AddNumberToObject(onlineack, "result",1);
        const char *const node_status ="node_status";
      //  cJSON_AddItemToObject(onlineack, node_status,js_body=cJSON_CreateObject());//js_body=cJSON_CreateArray()

          if (DeviceType==1) {
              LOG_I("DeviceType==1\n");
              cJSON_AddStringToObject(onlineack, "type", "A");
              cJSON_AddNumberToObject(onlineack, "node_count",(Device8StationsNum2)*8);
           //   cJSON_AddItemToArray(js_body, cJSON_CreateIntArray(ADeviceStatus, 8));
              cJSON_AddItemToObject(onlineack, "node_status",cJSON_CreateIntArray(ADeviceStatus, 8));


           } else if (DeviceType==2){
                      LOG_I("DeviceType==2\n");
                      //      cJSON_AddItemToArray(js_body, cJSON_CreateIntArray(BDeviceStatus, 8));
                      cJSON_AddStringToObject(onlineack, "type", "B");
                      cJSON_AddNumberToObject(onlineack, "node_count",(Device8StationsNum2)*8);
                   //   cJSON_AddItemToArray(js_body, cJSON_CreateIntArray(BDeviceStatus, 8));
                      cJSON_AddItemToObject(onlineack, "node_status",cJSON_CreateIntArray(BDeviceStatus, 8));

                    }else if (DeviceType==3) {
                           LOG_I("DeviceType==3\n");
                           cJSON_AddStringToObject(onlineack, "type", "C");
                           cJSON_AddNumberToObject(onlineack, "node_count",(Device8StationsNum2)*8);
                         //  cJSON_AddItemToArray(js_body, cJSON_CreateIntArray(CDeviceStatus, 6));
                           cJSON_AddItemToObject(onlineack, "node_status",cJSON_CreateIntArray(CDeviceStatus, 6));
                           }
          cJSON_AddNumberToObject(onlineack, "i",ia);
          cJSON_AddNumberToObject(onlineack, "u",12);
          cJSON_AddNumberToObject(onlineack, "t",temperature/10.0);
          cJSON_AddNumberToObject(onlineack, "h",humidity/10);
          cJSON_AddNumberToObject(onlineack, "s",signalstrength[0]);

        char *s0 = cJSON_PrintUnformatted(onlineack);
              PublishMessage(s0);//将接收到指令的回执发送出去
              if(s0){
                      LOG_I("ackOK json is %s\n",s0);
                      free(s0);
                   }
             cJSON_Delete(onlineack); //
             at_delete_resp(resp);

    /*--------------------------------------------------------------------------*/
    /*上电启动的时候，需要上传4号命令，*/
}

/**
49  * Function    ota_app_vtor_reconfig
50  * Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
51 */
/*static int ota_app_vtor_reconfig(void)
{
    #define NVIC_VTOR_MASK   0x3FFFFF80
    // Set the Vector Table base location by user application firmware definition
    SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

    return 0;
}*/
//INIT_BOARD_EXPORT(ota_app_vtor_reconfig);

static void idle_hook(void)
{
    /* 在空闲线程的回调函数里喂狗 */
    rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);

}
 int main(void)
{
     int status1=0;
     int status2=0;
     int status3=0;
     int EC20_SUBSCRIBE_RETRY_TIME =3;
     int retry =0;
     int retrytimes =0;
     int ConnectedMQTT =0; //ConnectedMQTT用于指示 上电之初 的MQTT是否连接上，与MQTTConnected不一样


    /*配置PVD,可以在掉电瞬间保存一写数据*/
    PVD_Config();

//   fal_init();
    LOG_I("The current version of APP Firmware is :%s\n",APP_VERSION);
    LOG_I("The clientID is :%s\n",clientid);

    /*检测出传感器所在位置的温度及湿度的数据，并存储，在连上服务器后进行上传*/
//       cat_dhtxx();

    //蜂鸣器引脚设置为输出模式，方便测试设备类型时候报警
    rt_pin_mode(BEEPPIN, PIN_MODE_OUTPUT);

    //4G模块的复位引脚设置为输出模式，方便后期的复位
     rt_pin_mode(GRESET, PIN_MODE_OUTPUT);

    //将4G模块复位一下，免得后面无法操作，且后续可以在看门狗中复位模块。
   /* rt_thread_mdelay(2000);
    rt_pin_write(GRESET, PIN_HIGH);
    rt_thread_mdelay(200);
    LOG_D("GRESET!");
    rt_pin_write(GRESET, PIN_LOW);*/
    rt_pin_write(GRESET, PIN_LOW);

    /*刚进入系统的时候响1声，因为后面有个软件复位CPU，这里是进入主函数的提示*/
    BEEP(1, 100, 500);

    /*先选择设备类型，未选择的时候返回1，蜂鸣器连续响两声，会持续报警，直到拨码开关搬动到正确的位置*/
    while(DetectDeviceType())
    {
        LOG_D("Please Select the Device Type!");
        BEEP(2, 100, 500);
        rt_thread_mdelay(5000);
    }

    /*根据上一步拨码开关的位置，判定设备类型，然后设置对应设备类型的引脚*/
    ret = SetDeviceTypePins(DeviceType);

    /*根据上一步函数探测到的设备类型，发送指令探测安装的八站模块的数量*/
    Device8StationsNum = DetectDevice8StationNum(DeviceType);
    Device8StationsNum2 = Device8StationsNum;
    LOG_D(" the Device Type IS %d!,the DetectDevice8StationNum is %d",DeviceType,Device8StationsNum-1);

    /*检测出传感器所在位置的温度及湿度的数据，并存储，在连上服务器后进行上传*/
    //cat_dhtxx();

    /*延时让系统在EC20线程初始化完成*/
    rt_thread_mdelay(2000);

    LOG_D(" the errcode IS ----------------------%d!",errcode);

    /*错误代码及对应的意义：
     * 2---没有选择设备的类型，蜂鸣器响2声，间隔5S；
     * 3---EC20模块没有安装；或者是安装不牢固；或者是串口的拨动开关S2的位置错误；蜂鸣器响3声，间隔8S；
     * 4---EC20模块没有安装天线或者天线接触不良；EC20模块所在位置无网络信号；蜂鸣器响4声，间隔8S；
     * 5---EC20模块无法注册GSM或者GPRS;SIM卡无效或者SIM卡欠费停机；蜂鸣器响5声，间隔8S；
     * 以上几个错误（2-5）一旦出现，都意味着需要断电后操作，不能带电插拔，所以此处陷入死循环，必须断电后执行操作，重新上电来重启*/
    while(errcode != 0)
    {
        if (errcode==3) {
            LOG_I("The EC20 IS Not On Line!,please check the EC20 Device Or the Uart3!");
            BEEP(3, 100, 500);
            rt_thread_mdelay(3000);
        } else if (errcode==4) {
            LOG_I("The EC20 signal strength check failed!,please check the Antenna !");
            BEEP(4, 100, 500);
            rt_thread_mdelay(3000);
        } else if (errcode==5) {
            LOG_I("The EC20 device GSM Or GPRS is register failed!,please check the Network Operator !");
            BEEP(5, 100, 500);
            rt_thread_mdelay(3000);
        }
        rt_thread_mdelay(5000);
    }

    LOG_E("OperatorCode------------: %d",  OperatorCode);

    /*获取设备中的sim卡电话号码（即正常使用的手机号码），或者是ICCID号码，作为设备ID号码，如果读取不到则报警，依据不同的运营商的物联网卡取不同的位数作为ID号码*/
      while(GetDeviceID())
    {
        LOG_D("can not get the own numbers(used for the deviceID) from the SIM!,please check the SIM Card!");
        BEEP(6, 100, 500);
        rt_thread_mdelay(5000);
    }

      /*开始开启alarm，方便后续的定时关闭站点-放在接收case1指令的时候开启*/
    //  alarm_initandstart();

      if (OperatorCode == 0) {
          LOG_E("OperatorCode------------=0!");   //是中国联通或者是中国电信，需要将最后一位去除
          strncpy(device_id3,device_id,strlen(device_id)-1);
          LOG_E("device_id after operation ------------%s!",device_id3);
    } else if (OperatorCode == 1) {
        LOG_E("OperatorCode------------=1!");
        strcpy(device_id3,device_id);
        LOG_E("device_id after operation ------------%s!",device_id3);
    }
     // strcpy(device_id2,device_id);
   //   strcpy(device_id3,device_id);

      //message = strcat(device_id2,message);上电的时候需要发送指令4，并将其中的operation_id = 0,这里不需要"is on line"了

    /*打开MQTT端口，连接到云端，订阅主题，发送上线消息（也就是4号指令），告知服务器设备上线了*/
    status1=OpenConnect(host_name,port,clientid,Username,Password);
    rt_thread_mdelay(100);
    LOG_E("1status1: %d",  status1);
    status2=ConnectToCloud(host_name,port,clientid,Username,Password);
    rt_thread_mdelay(500);
    LOG_E("1status2: %d",  status2);
    status3=SubscribeToTopic(cliend_idx,messageid,recvtopic,qos);
    rt_thread_mdelay(100);
  //  status3=1;
    LOG_E("1status3: %d",  status3);

    if ((status1==0)&& (status2==0)&&(status3==0)){
        BEEP(1, 1000, 100); //连接成功后，响一声1S的长时间，提示已经连接成功
        ConnectedMQTT =1 ;
        MQTTConnected =0 ;//这里证明上电之初，MQTT的连接是成功的了，如果中间出现了MQTT的断开，在中间ec20_urc_qmtstat_func会出现设置 MQTTConnected =1
    }
    else {
            while(((status1!=0)||(status2!=0)||(status3!=0)) && retry < EC20_SUBSCRIBE_RETRY_TIME)//
                   {
                        LOG_D(" EC20_SUBSCRIBE_RETRY_TIME +1");
                        retry++;
                        LOG_D(" retry=%d",retry);
                        rt_thread_mdelay(3000);

                       /* 将EC20模块进行复位，注意延时需要足够，不然后面的重连可能会失败
                        rt_pin_write(GRESET, PIN_HIGH);
                        rt_thread_mdelay(400);
                        LOG_D("GRESET!");
                        rt_pin_write(GRESET, PIN_LOW);

                        rt_thread_mdelay(15000);


                       status1=OpenConnect(host_name,port,clientid,Username,Password);
                       LOG_E("2status1: %d",  status1);
                       rt_thread_mdelay(100);
                       status2=ConnectToCloud(host_name,port,clientid,Username,Password);
                       LOG_E("2status2: %d",  status2);
                       rt_thread_mdelay(500);
                       status3=SubscribeToTopic(cliend_idx,messageid,recvtopic,qos);
                       LOG_E("2status3: %d",  status3);
                     //  status3=1;
                       LOG_E("2----status3: %d",  status3);
                       rt_thread_mdelay(100);*/
                        status1 = ReConnectAndSubscribe();
                      if (status1==0) {
                          LOG_I("mqtt failed ，But reconnect success！");
                            BEEP(1, 1000, 100); //连接成功后，响一声1S的长时间，提示已经连接成功
                            break;
                         }

                      /* if ((status1==0)&& (status2==0)&&(status3==0)){
                              LOG_I("mqtt失败了以后，但是重新连接成功！");
                               BEEP(1, 1000, 100); //连接成功后，响一声1S的长时间，提示已经连接成功
                               break;
                           }*/

                       if (retry==EC20_SUBSCRIBE_RETRY_TIME) {
                              LOG_I("mqtt failed，reset EC20,reconnected 3 times all failed,start to reboot system!！");
                              rt_thread_mdelay(1000);
                              rt_hw_cpu_reset();/*重新连接3次都不成功了以后，尝试将系统进行重启*/
                           }
                   }
             // LOG_I("看一下break以后这里是否执行！！！！！！！！！");
              ConnectedMQTT =1;
          }

    /* 添加多种 URC 数据至 URC 列表中，当接收到同时匹配 URC 前缀和后缀的数据，执行 URC 函数  */
    at_set_urc_table(urc_table, sizeof(urc_table) / sizeof(urc_table[0]));

    /*上电启动的时候，需要上传4号命令，*/
    if (ConnectedMQTT ==1) {
        PowerOnSendMessage();
    } else {

    }

    /*//A设备测试
            for (int var = 32; var < 48; var++) {
                      SendADeviceData(var+1,1);
                        rt_thread_mdelay(100);
               }
            for (int var = 32; var < 48; var++) {
                            SendADeviceData(var+1,2);
                              rt_thread_mdelay(100);
              }*/

   //B设备测试
  /* for (int var = 0; var < 2; var++) {
          SendBDeviceData(var+1,1);
            rt_thread_mdelay(100);
   }

   for (int var = 0; var < 2; var++) {
                 SendBDeviceData(var+1,2);
                   rt_thread_mdelay(100);
   }*/

 /*   //C设备测试
    for (int var = 0; var < 10; var++) {
        SendCDeviceData(var+1,1);
        rt_thread_mdelay(1);
    }

    for (int var = 0; var < 10; var++) {
        SendCDeviceData(var+1,2);
        rt_thread_mdelay(1);
    }
    rt_pin_write(CDeviceSelOUTPUT, PIN_LOW);
    rt_pin_write(C4245ENABLE, PIN_HIGH);*/

   /*  创建服务器响应结构体，64 为用户自定义接收数据最大长度
    resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));

     发送数据到服务器，并接收响应数据存放在 resp 结构体中
    at_exec_cmd(resp, "ATI");

     解析获取串口配置信息，1 表示解析响应数据第一行，'%*[^=]'表示忽略等号之前的数据
    at_resp_parse_line_args(resp, 1,"%s", &buf);

    LOG_E("baudrate=%s", &buf);
    LOG_E("baudrate=%s!");
     删除服务器响应结构体
    at_delete_resp(resp);*/
   /* rt_pin_mode(PKG_USING_DHTXX_SAMPLE_PIN, PIN_MODE_OUTPUT);
    while (1)
    {

    rt_pin_write(PKG_USING_DHTXX_SAMPLE_PIN, PIN_LOW);
    rt_thread_mdelay(5);
    rt_pin_write(PKG_USING_DHTXX_SAMPLE_PIN, PIN_HIGH);
    rt_thread_mdelay(10);

    rt_pin_write(PKG_USING_DHTXX_SAMPLE_PIN, PIN_LOW);
    rt_hw_us_delay(5);
        rt_pin_write(PKG_USING_DHTXX_SAMPLE_PIN, PIN_HIGH);
        rt_hw_us_delay(10);

        rt_pin_write(PKG_USING_DHTXX_SAMPLE_PIN, PIN_LOW);
           rt_hw_us_delay(2);
               rt_pin_write(PKG_USING_DHTXX_SAMPLE_PIN, PIN_HIGH);
               rt_hw_us_delay(1);
    }*/

    /* 根据设备名称查找看门狗设备，获取设备句柄 */
        wdg_dev = rt_device_find(WDT_DEVICE_NAME);
        if (!wdg_dev)
        {
            rt_kprintf("find %s failed!\n", WDT_DEVICE_NAME);
            return RT_ERROR;
        }

        /* 初始化设备 */
        rt_device_init(wdg_dev);

        /* 设置看门狗溢出时间 */
        ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
        if (ret != RT_EOK)
        {
            rt_kprintf("set %s timeout failed!\n", WDT_DEVICE_NAME);
            return RT_ERROR;
        }
        /* 启动看门狗 */
        ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
        if (ret != RT_EOK)
        {
            rt_kprintf("start %s failed!\n", WDT_DEVICE_NAME);
            return -RT_ERROR;
        }
        /* 设置空闲线程回调函数 ,在空闲线程回调函数中喂狗*/
            rt_thread_idle_sethook(idle_hook);


    while (1)
    {
        rt_thread_mdelay(60000*3); //这个延时的周期要大于EC20线程中的查询周期，不然没有意义，相当于线程中还没有查询最新的状态，这里就开始判定断网了

        LOG_D("every 3 minutes !");
        LOG_D("NetConnected= %d",NetConnected);
        LOG_D("MQTTConnected= %d",MQTTConnected);

        /*如果(NetConnected == 1)||(MQTTConnected ==1)，证明是网络断掉了或者是MQTT的连接断掉了，MQTT的连接需要 首先 依赖网络的连接上*/
        while (((NetConnected == 1)||(MQTTConnected ==1))&& retrytimes < 2 ){
                status1 =0;
                status1 = ReConnectAndSubscribe();
                 if (status1==0) {
                     LOG_I("(NetConnected == 1)||(MQTTConnected ==1),but MQTT reconnect SUCCESS!!");
                       BEEP(1, 1000, 100); //连接成功后，响一声1S的长时间，提示已经连接成功
                       NetConnected = 0;
                       MQTTConnected =0;
                       retrytimes = 0;
                       break;
                    }else {
                        retrytimes++;
                        LOG_D("retrytimes++,retrytimes = %d",retrytimes);
                        rt_thread_mdelay(3000);//这里值当检测线程中间隔是30s的时候，这里的间隔是40s;这里的值要大于检测线程中的值，保证这个延时过程中有刷新重检验过程

                        /*不能单纯关闭串口，再打开无响应了*/
                       /* rt_device_t serial= rt_device_find("uart3");
                        ret = rt_device_close(serial);
                        if (ret== RT_EOK) {
                            LOG_D("rt_device_close(serial) success!");
                            ret=rt_device_open(serial,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
                            if (ret== RT_EOK) {
                                LOG_D("rt_device_open(serial) success!");
                            } else {
                                LOG_D("rt_device_open(serial) failed!");
                            }
                        }else {
                            LOG_D("rt_device_close(serial) failed!");
                        }*/

                    }
            }
        if (retrytimes==2 ) {
            LOG_I("the NET Signal lost OR MQTT connect failed,tried 2 times but all failed!!");//网络断掉了或者是MQTT的连接断掉了,尝试了5次连接都无法成功连接上
            BEEP(7, 100, 500);
            rt_thread_mdelay(4000);
            BEEP(7, 100, 500);
            rt_thread_mdelay(4000);
            BEEP(7, 100, 500);
            rt_thread_mdelay(4000);
            retrytimes=0;

            rt_hw_cpu_reset();/*重新连接5次都不成功了以后，尝试将系统进行重启*/
        } else {
            LOG_D("Net is OK");
        }

    }
    return RT_EOK;
}

