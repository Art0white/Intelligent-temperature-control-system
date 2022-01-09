#include<reg52.h> 
#include<intrins.h> 
#include<math.h> 
#include<string.h>

struct PID { 
	unsigned int SetPoint;      // 设定目标 Desired Value 
	unsigned int Proportion;    // 比例常数 Proportional Const 
	unsigned int Integral;      // 积分常数 Integral Const 
	unsigned int Derivative;    // 微分常数 Derivative Const 
	unsigned int LastError;     // Error[-1] 
	unsigned int PrevError;     // Error[-2] 
	unsigned int SumError;      // Sums of Errors 
}; 
/**struct set_temper {
	unsigned int stvalue;
	unsigned char stflag;	
}set_temper_max={40, 0},set_temper_min{10 , 1};
**/
struct PID spid; 		// PID Control Structure 控制结构
unsigned int rout; 	// PID Response (Output) 响应输出
unsigned int rin; 	// PID Feedback (Input)  反馈输入
unsigned char high_time, low_time,count=0;   //占空比调节参数
#define uchar unsigned char
#define uint unsigned int
#define CLEAR_BIT(x, bit) (x &= ~(1 << bit))		//CLR
#define	SET_BIT(x, bit)	(x |= (1 << bit))				//SETB

/**************************************/
/**    定义特殊功能寄存器的位变量    **/
/**************************************/
sbit output=P2^7;
sbit ds=P3^2;
sbit DQ=P3^2;						//ds18b20与单片机连接口
sbit ledred=P2^6;
sbit ledgreen=P2^5;

uchar set[2]={0};
uchar n,num;
int set_temper_max=40, set_temper_min=10, temper, temp; 	//温度变量定义
int temper, temp;
uchar temp1; 										//按键标志
unsigned int s;                 //传递给rin，值为0，代表要求误差为0
float f_temp;                  	//转换后的温度

uint tvalue;         
uchar tflag;										//温度正负标志

uchar code LEDzf[]=             //LED显示正负
{
	0x00,0x40
};
uchar code LEDData[]= 					//LED显示数字0~9
{
 	0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f
};

uchar t3num;										//三种温度的标识

/********************/
/**    延时函数    **/
/********************/
void delay(i)
{
	uint j;
	for(i;i>0;i--)
		for(j=111;j>0;j--);
}

/**********************************/
/**     按照时序操作的初始化     **/
/**********************************/
void init()
{	
		t3num = 1;
}

/*************************DS1820程序****************************/

/************************/
/**      延时1微秒     **/
/************************/
void delay_18B20(unsigned int i)
{
   while(i--);
}

/************************/
/**     ds1820复位     **/
/************************/
void ds1820rst(void)
{ 
	unsigned char x=0;
	DQ = 1;          		//DQ复位
	delay_18B20(4);   	//延时
	DQ = 0;          		//DQ拉低
  TR0 = 0;
	delay_18B20(100); 	//精确延时大于
  TR0 = 1;
	DQ = 1;          		//拉高
	delay_18B20(40); 
} 

/********************/
/**     读数据     **/
/********************/
uchar ds1820rd(void)
{ 
	unsigned char i=0;
	unsigned char dat = 0;
  TR0=0;
	for (i=8;i>0;i--)
	{   
		DQ = 0; 					//给脉冲信号
		dat>>=1;
		DQ = 1; 					//给脉冲信号
		if(DQ)
		dat|=0x80;
		delay_18B20(10);
	}
  return(dat);
}

/********************/
/**     写数据     **/
/********************/
void ds1820wr(uchar wdata)
{
	unsigned char i=0;
  TR0=0;
  for (i=8; i>0; i--)
  { 
		DQ = 0;
		DQ = wdata&0x01;
		delay_18B20(10);
		DQ = 1;
		wdata>>=1;
  }
}

/**********************/
/**     获取温度     **/
/**********************/
uint get_temper()
{  
     
	uchar a,b;

	ds1820rst();    
	ds1820wr(0xcc);			//跳过读序列号
	ds1820wr(0x44);			//启动温度转换
	ds1820rst();    
	ds1820wr(0xcc);			//跳过读序列号
	ds1820wr(0xbe);			//读取温度
	a=ds1820rd();
	b=ds1820rd();
   
	tvalue=b;
	tvalue<<=8;
	tvalue=tvalue|a;		//
  TR0=1;
  if(tvalue<0x0fff)
		tflag=0;
  else 
	{
		tvalue=~tvalue+1;
		tflag=1;
	}
	tvalue=tvalue*(0.625);			//温度值扩大10倍，精确到1位小数
	temp=tvalue;
	return temp;
}

/**********************/
/**     显示温度     **/
/**********************/
void show_3temp(int t)
{
	uchar tflaguse;
	int d0,d1,d2,d3;
	int dt;
	if(t3num == 1)
	{
		tflaguse = tflag;
		dt = t;
	}
	else
	{
		if(t >= 0)
		{
			tflaguse = 0;
			dt = t;
		}
		else
		{
			tflaguse = 1;
			dt = -t;
		}
	}
	d0=dt/1000;
	d1=dt%1000/100;
	d2=dt%100/10;
	d3=dt%10;
		
	CLEAR_BIT(P2, 3);
	//P2 = 0xf7;
	P0 = LEDData[d3];
	delay(3);
	SET_BIT(P2, 3);
		
	CLEAR_BIT(P2, 2);
	//P2 = 0xfb;
	P0 = LEDData[d2];
	SET_BIT(P0, 7);
	delay(3);
	SET_BIT(P2, 2);
		
	CLEAR_BIT(P2, 1);
	//P2 = 0xfd;
	P0 = LEDData[d1];
	delay(3);
	SET_BIT(P2, 1);
		
	CLEAR_BIT(P2, 0);
	//P2 = 0xfe;
	P0 = LEDzf[tflaguse];
	delay(4);
	SET_BIT(P2, 0);
	
}
void dis_temp(int t)
{
	if(t3num == 1)
	{
		CLEAR_BIT(P2, 4);
		P0 = 0x06;
		delay(5);
		SET_BIT(P2, 4);
		show_3temp(t);
	}
	else if(t3num == 2)
	{
		CLEAR_BIT(P2, 4);
		P0 = 0x5b;
		delay(5);
		SET_BIT(P2, 4);
		show_3temp(set_temper_max*10);
	}
	else if(t3num == 3)
	{
		CLEAR_BIT(P2, 4);
		P0 = 0x4f;
		delay(5);
		SET_BIT(P2, 4);
		show_3temp(set_temper_min*10);
	}
}

/******************************4×4 键盘******************************/
/************************/
/**        键盘        **/
/************************/
void keyscan()//键盘扫描
{  
	unsigned char i, num = 16;  						//如果让num初始化为0-15之间的数，则没有按下键盘也会显示num的初始值
	for(i = 0; i < 4; i++)  								//行扫描
	{
		P1 = _crol_(0xfe,i);  								//高7位都给高，第一位给低(列全高，第一行给低)(循环左移实现)
		temp1 = P1;  													//temp采集P1信号(P1口高4位是输口，低4位是输出口)
		temp1 = temp1 & 0xf0;  								//屏蔽低4位输出信号，采集高4位输入信号
		if(temp1 != 0xf0)  										//若高四位都为1，则没有被按下；反之有按键被按下
		{
			delay(20);  												//延时20毫秒左右，之后重新采集(消抖)
			temp1 = P1;
			temp1 = temp1 & 0xf0;
			if(temp1 != 0xf0)  									//再次判断是否有按键被按下
			{
				temp1 = P1;  											//再次采集(避免重新判断扫描的是第几行按键)，准备判断按键号
				switch(temp1)  										//判断哪个按键被按下
				{
					case 0xee:
						t3num = 1;
						break;
					case 0xde:
						break;
					case 0xbe:
						break;
					case 0x7e:
						break;
					
					case 0xed:
						t3num = 2;
						break;
					case 0xdd:
						if(t3num == 2)
							set_temper_max++;
						break;
					case 0xbd:
						if(t3num == 2)
							set_temper_max--;
						break;
					case 0x7d:
						break;
					
					case 0xeb:
						t3num = 3;
						break;
					case 0xdb:
						if(t3num == 3)
							set_temper_min++;
						break;
					case 0xbb:
						if(t3num == 3)
							set_temper_min--;
						break;
					case 0x7b:
						break;
					
					case 0xe7:
						break;
					case 0xd7:
						break;
					case 0xb7:
						break;
					case 0x77:
						break;
					
					default: 
						break;
				}
				while((temp1 & 0xf0) != 0xf0)    		//判断按键是否弹起
				{
					temp1 = P1;
					temp1 = temp1 & 0xf0;
				}
			}
		}
	}
}

/************************/
/**     PID初始化      **/
/************************/
void PIDInit (struct PID *pp) 
{ 
	memset (pp,0,sizeof(struct PID)); 			//用参数0初始化pp
} 

/************************/
/**      PID计算       **/
/************************/
unsigned int PIDCalc(struct PID *pp, unsigned int NextPoint )
{ 
	unsigned int dError,Error; 
	Error = pp->SetPoint - NextPoint; 			//偏差 
	pp->SumError += Error; 									//积分 
	dError = pp->LastError - pp->PrevError; //当前微分 
	pp->PrevError = pp->LastError; 
	pp->LastError = Error; 
	return (pp->Proportion * Error					//比例
	+ pp->Integral * pp->SumError  					//积分项
	+ pp->Derivative * dError); 						//微分项 
} 

/*******************************************************/ 
/**               温度比较处理子程序                  **/
/*******************************************************/

void compare_temper(void) 
{ 

	unsigned char i; 
	temper = get_temper()/10;
	if(set_temper_min>temper) 								//设置温度大于当前温度
	{
		ledred=0;
		ledgreen=1;
		if(set_temper_min-temper>1) 						//温度相差1度以上
		{	 
			high_time=100;
			low_time=0;
		} 
		else 																		//设置温度不大于当前温度
		{ 
			for(i=0;i<10;i++) 
			{ 
				get_temper(); 
				rin = s; 														// Read Input 
				rout = PIDCalc ( &spid,rin ); 			//Perform PID Interation 
			} 
			if (high_time<=100) 	high_time=(unsigned char)(rout/10000); 
			else	high_time=100; 
			low_time= (100-high_time); 
			delay_18B20(498);
		}
	}
	else if(set_temper_max<=temper) 					//设置温度不大于当前温度
	{ 
		ledred=1;
		ledgreen=0;
		if(temper-set_temper_max>0) 						//温度相差0度以上
		{ 
			high_time=0; 
			low_time=100; 
		} 
		else 
		{ 
			for(i=0;i<10;i++) 
			{ 
				get_temper(); 
				rin = s; // Read Input 
				rout = PIDCalc ( &spid,rin ); // Perform PID Interation 
			} 
			if (high_time<100) high_time=(unsigned char)(rout/10000); 
			else 	high_time=0; 
			low_time= (100-high_time);
		}
	}
	else
	{
		ledred=1;
		ledgreen=1;
	}
}
/**void compare_temper(void) 
{ 
	temper = get_temper()/10;
	if(set_temper_min>=temper) 								//设置温度大于当前温度
	{
		ledred=0;
		ledgreen=1;
	} 
	else if(set_temper_max<temper) 					//设置温度不大于当前温度
	{ 
		ledred=1;
		ledgreen=0;
	}
	else
	{
		ledred=1;
		ledgreen=1;
	}
}**/
/***********************************************************************************/ 
/**           T0中断服务子程序，用于控制电平的翻转 ,40us*100=4ms周期              **/
/***********************************************************************************/ 
void serve_T0() interrupt 1 using 1 
{ 
	if(++count<=(high_time)) 	output=0;
	else if(count<=100) 
	{ 
		output=1; 
	} 
	else count=0; 
	TH0=0x2f; 
	TL0=0x40; 
}


/***********主函数**********/
void main(void)
{
	/**unsigned char i;**/
	init();//LED初始化
	TMOD=0x01;
	TH0=0x2f; 
	TL0=0x40;
	EA=1;
	ET0=1;
	TR0=1;
	high_time=50; 
	low_time=50;
	PIDInit ( &spid ); 							// Initialize Structure 
	// Set PID Coefficients
	spid.Proportion = 10; 					//	P 10
	spid.Integral = 8; 							//	I	8
	spid.Derivative = 6; 						//	D	6
	spid.SetPoint = 100; 						// Set PID Setpoint 
	while(1)
	{
		delay(1);
		keyscan(); 										//按键扫描
		dis_temp(get_temper());				//显示温度值
		compare_temper(); 						//比较温度
	}
}