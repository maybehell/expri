/**************************************************************************************************
* 文件名：HumiTempLight_RF.c
* 功能：无线温湿度采集实验
* 描述：
*   1. 节点A（发送端）：读取SHT10温湿度，通过无线发送。
*   2. 节点B（接收端）：接收无线数据，显示在LCD上。
**************************************************************************************************/
#include "hal.h"
#include "LCD.h"
#include "stdio.h"
#include "hal_rf.h"
#include "basic_rf.h"
#include "hal_int.h"
#include "hal_mcu.h"

// === RF 配置参数 ===
#define RF_CHANNEL                25      // 频道
#define PAN_ID                    0x1000  // 网络ID
#define SENDER_ADDR               0x0001  // 发送节点地址
#define RECEIVER_ADDR             0x0002  // 接收节点地址
#define PAYLOAD_LENGTH            2       // 数据长度：[0]温度 [1]湿度

// === SHT10 定义 ===
#define noACK 0
#define ACK   1
#define SCL          P1_0     
#define SDA          P1_1     

// === 全局变量 ===
static basicRfCfg_t basicRfConfig;
static uint8 pTxData[PAYLOAD_LENGTH];
static uint8 pRxData[PAYLOAD_LENGTH];

unsigned char d1,d2,d3;

// === SHT10 驱动函数声明 ===
void Wait(unsigned int ms);
void QWait(void);
char s_write_byte(unsigned char value);
char s_read_byte(unsigned char ack);
void s_transstart(void);
void s_connectionreset(void);
char s_measure( unsigned char *p_checksum, unsigned char mode);
void initIO(void);
void th_read(int *t,int *h);

// === 驱动函数实现 (保持原样，略做整理) ===
void Wait(unsigned int ms) {
   unsigned char g,k;
   while(ms) {
	  for(g=0;g<=167;g++) for(k=0;k<=48;k++);
      ms--;                            
   }
} 

void QWait() { asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP"); }

void initIO(void) {
  IO_DIR_PORT_PIN(1, 0, IO_OUT);
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  P1INP |= 0x03;
  SDA = 1; SCL = 0;
}

char s_write_byte(unsigned char value) { 
  unsigned char i,error=0;  
  IO_DIR_PORT_PIN(1, 0, IO_OUT); IO_DIR_PORT_PIN(1, 1, IO_OUT);
  for (i=0x80;i>0;i/=2) {
     if (i & value) SDA=1; else SDA=0;                        
    SCL = 1; QWait();QWait();QWait();QWait();QWait();
    SCL = 0; asm("NOP"); asm("NOP");
  }
  SDA = 1; 
  IO_DIR_PORT_PIN(1, 1, IO_IN); SCL = 1; asm("NOP");                          
  error = SDA; 
  QWait();QWait();QWait();
  IO_DIR_PORT_PIN(1, 1, IO_OUT); SDA = 1; SCL = 0;        
  return error;                                   
}

char s_read_byte(unsigned char ack) {
  IO_DIR_PORT_PIN(1, 0, IO_OUT); IO_DIR_PORT_PIN(1, 1, IO_OUT);
  unsigned char i,val=0;
  SDA= 1;
  IO_DIR_PORT_PIN(1, 1, IO_IN);
  for (i=0x80;i>0;i/=2) {
    SCL = 1;
    if (SDA) val = (val | i);
    SCL = 0; QWait();QWait();QWait();QWait();QWait();
  }
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  SDA = !ack; SCL = 1; QWait();QWait();QWait();QWait();QWait();
  SCL = 0; SDA = 1;
  return val;
}

void s_transstart(void) {
   IO_DIR_PORT_PIN(1, 0, IO_OUT); IO_DIR_PORT_PIN(1, 1, IO_OUT);
   SDA = 1; SCL = 0; QWait();QWait();
   SCL = 1;QWait();QWait(); SDA = 0;QWait();QWait(); 
   SCL = 0;QWait();QWait();QWait();QWait();QWait();
   SCL = 1;QWait();QWait(); SDA = 1;QWait();QWait(); SCL = 0;QWait();QWait();
}

void s_connectionreset(void) {
  IO_DIR_PORT_PIN(1, 0, IO_OUT); IO_DIR_PORT_PIN(1, 1, IO_OUT);
  unsigned char i;
  SDA = 1; SCL= 0;
  for(i=0;i<9;i++) { SCL = 1;QWait();QWait(); SCL = 0;QWait();QWait(); }
  s_transstart();
}

char s_measure( unsigned char *p_checksum, unsigned char mode) {
  unsigned er=0; unsigned int i,j;
  s_transstart();
  switch(mode) {
    case 3:er+=s_write_byte(3);break;
    case 5:er+=s_write_byte(5);break;
    default:break;
  }
  IO_DIR_PORT_PIN(1, 1, IO_IN);
  for(i=0;i<65535;i++) {
    for(j=0;j<100;j++) { if(SDA == 0) break; } // 缩短等待循环以防死锁
    if(SDA == 0) break;
  }
  if(SDA) er += 1;
  d1 = s_read_byte(ACK); d2 = s_read_byte(ACK); d3 = s_read_byte(noACK);
  return er;
}

void th_read(int *t,int *h ) {
  unsigned char error,checksum; float humi,temp; int tmp;
  initIO();
  s_connectionreset(); error=0;
  error+=s_measure(&checksum,5); humi = d1*256+d2;
  error+=s_measure(&checksum,3); temp = d1*256+d2;
  if(error!=0) s_connectionreset();
  else {      
     temp = temp*0.01 - 44.0;
     humi = (temp - 25) * (0.01 + 0.00008 * humi) -0.0000028 * humi * humi + 0.0405 * humi-4;
     if(humi>100) humi = 100; if(humi<0.1) humi = 0.1;
  }
  if((int)(temp*10)%10 > 4) temp+=1;
  *t=(int)temp;
  if((int)(humi*10)%10 > 4) humi+=1;
  *h=(int)humi;
}

// === 主程序逻辑 ===

// 发送节点逻辑 (Node A)
void appSender() {
    int tempera, humidity;
    char s[20];
    
    // 初始化 RF
    basicRfConfig.myAddr = SENDER_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
        GUI_PutString5_7(10, 50, "RF Init Failed"); LCM_Refresh(); while(1);
    }
    
    GUI_ClearScreen();
    GUI_PutString5_7(10, 5, "Mode: SENDER");
    GUI_PutString5_7(10, 25, "Reading SHT10...");
    LCM_Refresh();

    while(1) {
        // 1. 读取温湿度
        th_read(&tempera, &humidity);
        
        // 2. 本地显示 (可选，方便调试)
        sprintf(s, "T:%d C  H:%d %%", tempera, humidity);
        GUI_PutString5_7(10, 40, s);
        LCM_Refresh();
        
        // 3. 打包数据
        // 简单处理：直接强转为 uint8 发送 (假设数值在0-255范围内)
        pTxData[0] = (uint8)tempera;
        pTxData[1] = (uint8)humidity;
        
        // 4. 发送数据
        basicRfSendPacket(RECEIVER_ADDR, pTxData, PAYLOAD_LENGTH);
        
        // 5. 闪烁 LED 指示发送
        // 假设 LED1 在 P1_0 (注意这里SHT10也用了P1_0，可能有冲突，建议只用屏幕提示)
        // 或者使用板载的其他LED
        GUI_PutString5_7(80, 5, "TX->"); LCM_Refresh();
        Wait(50); // 短暂显示状态
        GUI_PutString5_7(80, 5, "    "); LCM_Refresh();
        
        // 6. 延时 (每秒发一次)
        Wait(1000); 
    }
}

// 接收节点逻辑 (Node B)
void appReceiver() {
    char s[20];
    int rcvTemp, rcvHumi;
    
    // 初始化 RF
    basicRfConfig.myAddr = RECEIVER_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
        GUI_PutString5_7(10, 50, "RF Init Failed"); LCM_Refresh(); while(1);
    }
    basicRfReceiveOn(); // 开启接收
    
    GUI_ClearScreen();
    GUI_PutString5_7(10, 5, "Mode: RECEIVER");
    GUI_PutString5_7(10, 25, "Waiting Data...");
    LCM_Refresh();
    
    while(1) {
        while(!basicRfPacketIsReady()) {
            // 等待数据时可以做点别的，或者空转
        }
        
        if(basicRfReceive(pRxData, PAYLOAD_LENGTH, NULL) > 0) {
            // 1. 解析数据
            rcvTemp = pRxData[0];
            rcvHumi = pRxData[1];
            
            // 2. 显示在 LCD 上
            GUI_ClearScreen();
            GUI_PutString5_7(10, 5, "Mode: RECEIVER");
            GUI_PutString5_7(80, 5, "<-RX"); // 提示收到数据
            
            GUI_PutString5_7(10, 25, "Remote Data:");
            
            sprintf(s, "Temp: %d C", rcvTemp);
            GUI_PutString5_7(20, 40, s);
            
            sprintf(s, "Humi: %d %%", rcvHumi);
            GUI_PutString5_7(20, 55, s);
            
            LCM_Refresh();
        }
    }
}

void main() {
    // 系统初始化
    halBoardInit(); // 确保 hal_board.c 里的初始化被调用
    GUI_Init();
    
    // RF 通用配置
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.channel = RF_CHANNEL;
    basicRfConfig.ackRequest = TRUE;
    
    // 菜单选择界面
    GUI_ClearScreen();
    GUI_PutString5_7(10, 10, "Select Mode:");
    GUI_PutString5_7(10, 30, "SW2: Receiver"); // 接收端
    GUI_PutString5_7(10, 45, "SW3: Sender");   // 发送端
    LCM_Refresh();
    
    // 等待按键
    // 注意：需要确保你的工程中有 halkeycmd 的定义
    // 如果没有 halkeycmd，可能需要直接读寄存器或用 HalKeyRead
    uint8 key = 0;
    while(1) {
        // 简单的按键轮询 (假设 SW2=P0_1, SW3=P1_2 等，根据你的 hal_board 定义调整)
        // 建议使用库提供的函数
        // key = HalKeyRead(); 
        // 假设这里用你之前代码里的 halkeycmd()
         extern uint8 halkeycmd(void); 
         key = halkeycmd();
         
         if (key == 2) { // SW2
             appReceiver();
             break;
         }
         if (key == 3) { // SW3
             appSender();
             break;
         }
    }
}