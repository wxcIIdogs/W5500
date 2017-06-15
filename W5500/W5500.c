/**********************************************************************************
W5500网络模块 主代码

**********************************************************************************/

#include "stm32f10x.h"
#include "spi.h"				
#include "W5500.h"
#include "delay.h"	


//网络参数变量定义 
u8 Gateway_IP[4];//网关IP地址 
u8 Sub_Mask[4];	//子网掩码 
u8 Phy_Addr[6];	//物理地址(MAC) 
u8 IP_Addr[4];	//本机IP地址 

u8 S0_Port[2];	//端口0的端口号(5000) 
u8 S0_DIP[4];	//端口0目的IP地址 
u8 S0_DPort[2];	//端口0目的端口号(6000) 

u8 UDP_DIPR[4];	    //UDP(广播)模式,目的主机IP地址
u8 UDP_DPORT[2];	//UDP(广播)模式,目的主机端口号


//端口的运行模式
u8 S0_Mode =3;	            //端口0的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
#define TCP_SERVER	0x00	//TCP服务器模式
#define TCP_CLIENT	0x01	//TCP客户端模式 
#define UDP_MODE	0x02	//UDP(广播)模式 

//端口的运行状态
u8 S0_State =0;	            //端口0状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
#define S_INIT		0x01	//端口完成初始化 
#define S_CONN		0x02	//端口完成连接,可以正常传输数据 

//端口收发数据的状态
u8 S0_Data;	               	//端口0接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成 
#define S_RECEIVE	 0x01	//端口接收到一个数据包 
#define S_TRANSMITOK 0x02	//端口发送一个数据包完成 

//端口数据缓冲区 
u8 Rx_Buffer[2048];	//端口接收数据缓冲区 
u8 Tx_Buffer[2048];	//端口发送数据缓冲区 

u8 W5500_Interrupt;	//W5500中断标志(0:无中断,1:有中断)







//W5500各端口初始化及配置中断模式
//相应配置说明：
//sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0
void W5500_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	

 	EXTI_InitTypeDef EXTI_InitStructure;			//中断配置	  即哪个中断线  EXTI_Line0-15
													//模式 EXTI_Mode_Interrupt中断  EXTI_Mode_Event 事件
													//触发方式  EXTI_Trigger_Falling 下降沿触发
													//			EXTI_Trigger_Rising	 上升沿触发
													//			EXTI_Trigger_Rising_Falling	  任意电平触发

 	NVIC_InitTypeDef NVIC_InitStructure;			//中断参数 中断优先级


  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟



	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTB时钟使能 


	// W5500_RST引脚初始化配置(PA2) 
	GPIO_InitStructure.GPIO_Pin  = W5500_RST;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_GPIO, &GPIO_InitStructure);
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);
	
	// W5500_INT引脚初始化配置(PA3) 	
	GPIO_InitStructure.GPIO_Pin = W5500_INT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(W5500_INT_GPIO, &GPIO_InitStructure);


	// 初始化网络模块SPI-CS引脚 (PA0)
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);



 	// Enable the EXTI3 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;				//W5500_INT所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;			//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);


  	// Connect EXTI Line3 to PA3 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);


	// PA3 as W5500 interrupt input 
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

}


//中断线3  PA3响应 W5500来的数据 并置一标志位
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);	//清中断线
		W5500_Interrupt=1;
	}
}



//通过SPI1发送一个字节
//dat 发送的字节
//无返回
void SPI1_Send_Byte(u8 dat)
{
	SPI_I2S_SendData(SPI1,dat);//写1个字节数据
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//死循环等待数据寄存器空
}



//PI1发送2个字节数据(16位)
//dat:待发送的16位数据
void SPI1_Send_Short(u16 dat)
{
	SPI1_Send_Byte(dat/256);//写数据高位   相当于dat>>8
	SPI1_Send_Byte(dat);	//写数据低位
}




//通过SPI1向W5500指定地址寄存器写1个字节数据
// reg:16位寄存器地址,dat:待写入的数据
void Write_W5500_1Byte(u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500

	SPI1_Send_Short(reg);   //通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_WRITE|COMMON_R); //通过SPI1写控制字节,1个字节数据长度,写数据,选择通用寄存器
	SPI1_Send_Byte(dat);  //写1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定地址寄存器写2个字节数据
//reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
void Write_W5500_2Byte(u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//通过SPI1写控制字节,2个字节数据长度,写数据,选择通用寄存器
	SPI1_Send_Short(dat);//写16位数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定地址寄存器写n个字节数据
//reg:16位寄存器地址,*dat_ptr:待写入数据缓冲区指针,size:待写入的数据长度
void Write_W5500_nByte(u16 reg, u8 *dat_ptr, u16 size)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500	
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(VDM|RWB_WRITE|COMMON_R);//通过SPI1写控制字节,N个字节数据长度,写数据,选择通用寄存器

	for(i=0;i<size;i++)//循环将缓冲区的size个字节数据写入W5500
	{
		SPI1_Send_Byte(*dat_ptr++);//写一个字节数据
	}

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定端口寄存器写1个字节数据
//s:端口号,reg:16位寄存器地址,dat:待写入的数据
void Write_W5500_SOCK_1Byte(SOCKET s, u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500	
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,1个字节数据长度,写数据,选择端口s的寄存器
	SPI1_Send_Byte(dat);//写1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}



//通过SPI1向指定端口寄存器写2个字节数据
//s:端口号,reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
void Write_W5500_SOCK_2Byte(SOCKET s, u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,2个字节数据长度,写数据,选择端口s的寄存器
	SPI1_Send_Short(dat);//写16位数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定端口寄存器写4个字节数据
//s:端口号,reg:16位寄存器地址,*dat_ptr:待写入的4个字节缓冲区指针
void Write_W5500_SOCK_4Byte(SOCKET s, u16 reg, u8 *dat_ptr)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,4个字节数据长度,写数据,选择端口s的寄存器

	SPI1_Send_Byte(*dat_ptr++);//写第1个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第2个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第3个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第4个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//拉高CS  取消片选
}



//读取W5500指定地址寄存器的1个字节数据
//reg:16位寄存器地址
//返回:读取到寄存器的1个字节数据
u8 Read_W5500_1Byte(u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);  //通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_READ|COMMON_R);//通过SPI1写控制字节,1个字节数据长度,读数据,选择通用寄存器

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//发送空数据 等待返回数据
	i=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//拉高CS  取消片选
	return i;//返回读取到的寄存器数据
}



//读W5500指定端口寄存器的1个字节数据
//s:端口号,reg:16位寄存器地址
//返回值:读取到寄存器的1个字节数据
u8 Read_W5500_SOCK_1Byte(SOCKET s, u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);   //通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//通过SPI1写控制字节,1个字节数据长度,读数据,选择端口s的寄存器

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);   //发送空数据 等待返回数据
	i=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //拉高CS  取消片选
	return i;//返回读取到的寄存器数据
}



//读W5500指定端口寄存器的2个字节数据
//s:端口号,reg:16位寄存器地址
//返回值:读取到寄存器的2个字节数据(16位)
u16 Read_W5500_SOCK_2Byte(SOCKET s, u16 reg)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//通过SPI1写控制字节,2个字节数据长度,读数据,选择端口s的寄存器

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//发送空数据 等待返回数据
	i=SPI_I2S_ReceiveData(SPI1);//读取高位数据
	SPI1_Send_Byte(0x00);//发送空数据 等待返回数据
	i*=256;
	i+=SPI_I2S_ReceiveData(SPI1);//读取低位数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//拉高CS  取消片选
	return i;//返回读取到的寄存器数据
}




//从W5500接收数据缓冲区中读取数据
//s:端口号,*dat_ptr:数据保存缓冲区指针
//返回值:读取到的数据长度,rx_size个字节
u16 Read_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr)
{
	u16 rx_size;
	u16 offset, offset1;
	u16 i;
	u8 j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;  //没接收到数据则返回
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);    //计算实际的物理地址

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500

	SPI1_Send_Short(offset);//写16位地址
	SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
	j=SPI_I2S_ReceiveData(SPI1);
	
	if((offset+rx_size)<S_RX_SIZE)//如果最大地址未超过W5500接收缓冲区寄存器的最大地址
	{
		for(i=0;i<rx_size;i++)//循环读取rx_size个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	else//如果最大地址超过W5500接收缓冲区寄存器的最大地址
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//循环读取出前offset个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
		GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //拉高CS  取消片选

		GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500

		SPI1_Send_Short(0x00);//写16位地址
		SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
		j=SPI_I2S_ReceiveData(SPI1);

		for(;i<rx_size;i++)//循环读取后rx_size-offset个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选

	offset1+=rx_size;//更新实际物理地址,即下次读取接收到的数据的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//发送启动接收命令
	return rx_size;//返回接收到数据的长度
}



//将数据写入W5500的数据发送缓冲区
//s:端口号,*dat_ptr:数据保存缓冲区指针,size:待写入数据的长度
void Write_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr, u16 size)
{
	u16 offset,offset1;
	u16 i;

	//如果是UDP模式,可以在此设置目的主机的IP和端口号
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) != SOCK_UDP)//如果Socket打开失败
	{		
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//设置目的主机IP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//设置目的主机端口号				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//计算实际的物理地址

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//置W5500的SCS为低电平

	SPI1_Send_Short(offset);//写16位地址
	SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

	if((offset+size)<S_TX_SIZE)//如果最大地址未超过W5500发送缓冲区寄存器的最大地址
	{
		for(i=0;i<size;i++)//循环写入size个字节数据
		{
			SPI1_Send_Byte(*dat_ptr++);//写入一个字节的数据		
		}
	}
	else//如果最大地址超过W5500发送缓冲区寄存器的最大地址
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//循环写入前offset个字节数据
		{
			SPI1_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
		GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //置W5500的SCS为高电平

		GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//置W5500的SCS为低电平

		SPI1_Send_Short(0x00);//写16位地址
		SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

		for(;i<size;i++)//循环写入size-offset个字节数据
		{
			SPI1_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
	}
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //置W5500的SCS为高电平

	offset1+=size;//更新实际物理地址,即下次写待发送数据到发送数据缓冲区的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//发送启动发送命令				
}



//W5500硬件复位
//说明：W5500的复位引脚保持低电平至少500us以上,才能使W5500进入复位状态
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);  //复位引脚拉低
	delay_ms(50);
	GPIO_SetBits(W5500_RST_GPIO, W5500_RST);    //复位引脚拉高
	delay_ms(200);
	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0); //死循环等待以太网连接完成	这里要将网线连接到路由和模块上 否则将无法通过
}




//初始化W5500相应寄存器
//说明：硬件初始化后要对其进行相应软件初始化
void W5500_Init(void)
{
	u8 i=0;

	Write_W5500_1Byte(MR, RST);  //软件复位W5500,置1有效,复位后自动清0
	delay_ms(10);                //延时10ms

	//设置网关(Gateway)的IP地址,Gateway_IP为4字节
	//使用网关可以使通信突破子网的局限，通过网关可以访问到其它子网或进入Internet
	Write_W5500_nByte(GAR, Gateway_IP, 4);
			
	//设置子网掩码(MASK)值,SUB_MASK为4字节
	//子网掩码用于子网运算
	Write_W5500_nByte(SUBR,Sub_Mask,4);		
	
	//设置物理地址,PHY_ADDR为6字节,用于唯一标识网络设备的物理地址值
	//该地址值需要到IEEE申请，按照OUI的规定，前3个字节为厂商代码，后三个字节为产品序号
	//如果自己定义物理地址，注意第一个字节必须为偶数
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//设置本机的IP地址,IP_ADDR为4字节
	//注意，网关IP必须与本机IP属于同一个子网，否则本机将无法找到网关
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//设置发送缓冲区和接收缓冲区的大小，参考W5500数据手册 初始化分别配置为2k 总共为16k
	for(i=0;i<8;i++)
	{
		Write_W5500_SOCK_1Byte(i,Sn_RXBUF_SIZE, 0x02);//Socket Rx memory size=2k
		Write_W5500_SOCK_1Byte(i,Sn_TXBUF_SIZE, 0x02);//Socket Tx mempry size=2k
	}

	//设置重试时间，默认为2000(200ms) 
	//每一单位数值为100微秒,初始化时值设为2000(0x07D0),等于200毫秒
	Write_W5500_2Byte(RTR, 0x07d0);

	//设置重试次数，默认为8次 
	//如果重发的次数超过设定值,则产生超时中断(相关的端口中断寄存器中的Sn_IR 超时位(TIMEOUT)置“1”)
	Write_W5500_1Byte(RCR,8);

	//启动中断，参考W5500数据手册确定自己需要的中断类型
	//IMR_CONFLICT是IP地址冲突异常中断,IMR_UNREACH是UDP通信时，地址无法到达的异常中断
	//其它是Socket事件中断，根据需要添加
	Write_W5500_1Byte(IMR,IM_IR7 | IM_IR6);	//中断允许寄存器
	Write_W5500_1Byte(SIMR,S0_IMR);			//socket中断状态0-7 寄存器配置在S0上
	Write_W5500_SOCK_1Byte(0, Sn_IMR, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);	//S0 中断功能全部打开
}




//检查网关服务器
//属于模拟一端口的相应操作，判断W5500模式是否进入可以通讯状态。
//返回值:成功返回TRUE(0xFF),失败返回FALSE(0x00)
u8 Detect_Gateway(void)
{
	u8 ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//检查网关及获取网关的物理地址
	Write_W5500_SOCK_4Byte(0,Sn_DIPR,ip_adde);//向目的地址寄存器写入与本机IP不同的IP值
	Write_W5500_SOCK_1Byte(0,Sn_MR,MR_TCP);   //设置socket为TCP模式
	Write_W5500_SOCK_1Byte(0,Sn_CR,OPEN);     //打开Socket	
	delay_ms(5);//延时5ms 	
	
	if(Read_W5500_SOCK_1Byte(0,Sn_SR) != SOCK_INIT)//如果socket打开失败	 即启动TCP模式失败
	{
		Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(0,Sn_CR,CONNECT);//设置Socket为Connect模式	TCP作为客户端					

	do
	{
		u8 j=0;
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//读取Socket0中断标志寄存器
		if(j!=0)						 //如果有中断提示 则写1清对应中断
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		delay_ms(5);//延时5ms 
		if((j&IR_TIMEOUT) == IR_TIMEOUT) //超时
		{
			return FALSE;	
		}
		else if(Read_W5500_SOCK_1Byte(0,Sn_DHAR) != 0xff)	 //只判断末位MAC 是否为FF 如果成功加载这里应不为FF
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//关闭Socket	   说明测试成功
			return TRUE;							
		}
	}while(1);
}




//指定Socket(0~7)初始化
//s:待初始化的端口
void Socket_Init(SOCKET s)
{
	//设置分片长度，参考W5500数据手册，该值可以不修改	
	Write_W5500_SOCK_2Byte(0, Sn_MSSR, 1460);//最大分片字节数=1460(0x5b4)
	//设置指定端口
	switch(s)
	{
		case 0:
			//设置端口0的端口号
			Write_W5500_SOCK_2Byte(0, Sn_PORT, S0_Port[0]*256+S0_Port[1]);
			//设置端口0目的(远程)端口号
			Write_W5500_SOCK_2Byte(0, Sn_DPORTR, S0_DPort[0]*256+S0_DPort[1]);
			//设置端口0目的(远程)IP地址
			Write_W5500_SOCK_4Byte(0, Sn_DIPR, S0_DIP);			
			
			break;

		case 1:
			break;

		case 2:
			break;

		case 3:
			break;

		case 4:
			break;

		case 5:
			break;

		case 6:
			break;

		case 7:
			break;

		default:
			break;
	}
}



//设置指定Socket(0~7)为客户端与远程服务器连接
//s:待设定的端口
//返回值:成功返回TRUE(0xFF),失败返回FALSE(0x00)
//说明  :当本机Socket工作在客户端模式时,引用该程序,与远程服务器建立连接
//	     如果启动连接后出现超时中断，则与服务器连接失败,需要重新调用该程序连接
//	     该程序每调用一次,就与服务器产生一次连接
u8 Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket
	delay_ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//设置Socket为Connect模式
	return TRUE;//返回TRUE,设置成功
}




//设置指定Socket(0~7)作为服务器等待远程主机的连接
//s:待设定的端口
//返回值:成功返回TRUE(0xFF),失败返回FALSE(0x00)
//说明  :当本机Socket工作在服务器模式时,引用该程序,等等远程主机的连接
//		 该程序只调用一次,就使W5500设置为服务器模式
u8 Socket_Listen(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket	
	delay_ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//设置Socket为侦听模式	
	delay_ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//如果socket设置失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//设置不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}

	return TRUE;

	//至此完成了Socket的打开和设置侦听工作,至于远程客户端是否与它建立连接,则需要等待Socket中断，
	//以判断Socket的连接是否成功。参考W5500数据手册的Socket中断状态
	//在服务器侦听模式不需要设置目的IP和目的端口号
}





//设置指定Socket(0~7)为UDP模式
//s:待设定的端口
//返回值:成功返回TRUE(0xFF),失败返回FALSE(0x00)
//说明  :如果Socket工作在UDP模式,引用该程序,在UDP模式下,Socket通信不需要建立连接
//		 该程序只调用一次，就使W5500设置为UDP模式
u8 Socket_UDP(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP);//设置Socket为UDP模式*/
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket*/
	delay_ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//如果Socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}
	else
		return TRUE;

	//至此完成了Socket的打开和UDP模式设置,在这种模式下它不需要与远程主机建立连接
	//因为Socket不需要建立连接,所以在发送数据前都可以设置目的主机IP和目的Socket的端口号
	//如果目的主机IP和目的Socket的端口号是固定的,在运行过程中没有改变,那么也可以在这里设置
}





//函数名:W5500_Interrupt_Process
//描述  :W5500中断处理程序框架
void W5500_Interrupt_Process(void)
{
	u8 i,j;

IntDispose:
	W5500_Interrupt=0;//清零中断标志
	i = Read_W5500_1Byte(IR);//读取中断标志寄存器
	Write_W5500_1Byte(IR, (i&0xf0));//回写清除中断标志

//	if((i & CONFLICT) == CONFLICT)//IP地址冲突异常处理
//	{
//		 //自己添加代码
//	}
//
//	if((i & UNREACH) == UNREACH)//UDP模式下地址无法到达异常处理
//	{
//		//自己添加代码
//	}

	i=Read_W5500_1Byte(SIR);//读取端口中断标志寄存器	
	if((i & S0_INT) == S0_INT)//Socket0事件处理 
	{
		j=Read_W5500_SOCK_1Byte(0,Sn_IR); //读取Socket0中断标志寄存器
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);//回写清除中断标志

		if(j&IR_CON)//在TCP模式下,Socket0成功连接 
		{
			S0_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//关闭端口,等待重新打开连接 
			Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
		if(j&IR_SEND_OK)//Socket0数据发送完成,可以再次启动S_tx_process()函数发送数据 
		{
			S0_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动S_rx_process()函数 
		{
			S0_Data|=S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);// 关闭端口,等待重新打开连接 
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
	}

	if(Read_W5500_1Byte(SIR) != 0) 
		goto IntDispose;
}

