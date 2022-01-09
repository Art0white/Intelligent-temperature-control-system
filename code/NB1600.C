#include<reg52.h> 
#include<intrins.h> 
#include<math.h> 
#include<string.h>

struct PID { 
	unsigned int SetPoint;      // �趨Ŀ�� Desired Value 
	unsigned int Proportion;    // �������� Proportional Const 
	unsigned int Integral;      // ���ֳ��� Integral Const 
	unsigned int Derivative;    // ΢�ֳ��� Derivative Const 
	unsigned int LastError;     // Error[-1] 
	unsigned int PrevError;     // Error[-2] 
	unsigned int SumError;      // Sums of Errors 
}; 
/**struct set_temper {
	unsigned int stvalue;
	unsigned char stflag;	
}set_temper_max={40, 0},set_temper_min{10 , 1};
**/
struct PID spid; 		// PID Control Structure ���ƽṹ
unsigned int rout; 	// PID Response (Output) ��Ӧ���
unsigned int rin; 	// PID Feedback (Input)  ��������
unsigned char high_time, low_time,count=0;   //ռ�ձȵ��ڲ���
#define uchar unsigned char
#define uint unsigned int
#define CLEAR_BIT(x, bit) (x &= ~(1 << bit))		//CLR
#define	SET_BIT(x, bit)	(x |= (1 << bit))				//SETB

/**************************************/
/**    �������⹦�ܼĴ�����λ����    **/
/**************************************/
sbit output=P2^7;
sbit ds=P3^2;
sbit DQ=P3^2;						//ds18b20�뵥Ƭ�����ӿ�
sbit ledred=P2^6;
sbit ledgreen=P2^5;

uchar set[2]={0};
uchar n,num;
int set_temper_max=40, set_temper_min=10, temper, temp; 	//�¶ȱ�������
int temper, temp;
uchar temp1; 										//������־
unsigned int s;                 //���ݸ�rin��ֵΪ0������Ҫ�����Ϊ0
float f_temp;                  	//ת������¶�

uint tvalue;         
uchar tflag;										//�¶�������־

uchar code LEDzf[]=             //LED��ʾ����
{
	0x00,0x40
};
uchar code LEDData[]= 					//LED��ʾ����0~9
{
 	0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f
};

uchar t3num;										//�����¶ȵı�ʶ

/********************/
/**    ��ʱ����    **/
/********************/
void delay(i)
{
	uint j;
	for(i;i>0;i--)
		for(j=111;j>0;j--);
}

/**********************************/
/**     ����ʱ������ĳ�ʼ��     **/
/**********************************/
void init()
{	
		t3num = 1;
}

/*************************DS1820����****************************/

/************************/
/**      ��ʱ1΢��     **/
/************************/
void delay_18B20(unsigned int i)
{
   while(i--);
}

/************************/
/**     ds1820��λ     **/
/************************/
void ds1820rst(void)
{ 
	unsigned char x=0;
	DQ = 1;          		//DQ��λ
	delay_18B20(4);   	//��ʱ
	DQ = 0;          		//DQ����
  TR0 = 0;
	delay_18B20(100); 	//��ȷ��ʱ����
  TR0 = 1;
	DQ = 1;          		//����
	delay_18B20(40); 
} 

/********************/
/**     ������     **/
/********************/
uchar ds1820rd(void)
{ 
	unsigned char i=0;
	unsigned char dat = 0;
  TR0=0;
	for (i=8;i>0;i--)
	{   
		DQ = 0; 					//�������ź�
		dat>>=1;
		DQ = 1; 					//�������ź�
		if(DQ)
		dat|=0x80;
		delay_18B20(10);
	}
  return(dat);
}

/********************/
/**     д����     **/
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
/**     ��ȡ�¶�     **/
/**********************/
uint get_temper()
{  
     
	uchar a,b;

	ds1820rst();    
	ds1820wr(0xcc);			//���������к�
	ds1820wr(0x44);			//�����¶�ת��
	ds1820rst();    
	ds1820wr(0xcc);			//���������к�
	ds1820wr(0xbe);			//��ȡ�¶�
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
	tvalue=tvalue*(0.625);			//�¶�ֵ����10������ȷ��1λС��
	temp=tvalue;
	return temp;
}

/**********************/
/**     ��ʾ�¶�     **/
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

/******************************4��4 ����******************************/
/************************/
/**        ����        **/
/************************/
void keyscan()//����ɨ��
{  
	unsigned char i, num = 16;  						//�����num��ʼ��Ϊ0-15֮���������û�а��¼���Ҳ����ʾnum�ĳ�ʼֵ
	for(i = 0; i < 4; i++)  								//��ɨ��
	{
		P1 = _crol_(0xfe,i);  								//��7λ�����ߣ���һλ����(��ȫ�ߣ���һ�и���)(ѭ������ʵ��)
		temp1 = P1;  													//temp�ɼ�P1�ź�(P1�ڸ�4λ����ڣ���4λ�������)
		temp1 = temp1 & 0xf0;  								//���ε�4λ����źţ��ɼ���4λ�����ź�
		if(temp1 != 0xf0)  										//������λ��Ϊ1����û�б����£���֮�а���������
		{
			delay(20);  												//��ʱ20�������ң�֮�����²ɼ�(����)
			temp1 = P1;
			temp1 = temp1 & 0xf0;
			if(temp1 != 0xf0)  									//�ٴ��ж��Ƿ��а���������
			{
				temp1 = P1;  											//�ٴβɼ�(���������ж�ɨ����ǵڼ��а���)��׼���жϰ�����
				switch(temp1)  										//�ж��ĸ�����������
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
				while((temp1 & 0xf0) != 0xf0)    		//�жϰ����Ƿ���
				{
					temp1 = P1;
					temp1 = temp1 & 0xf0;
				}
			}
		}
	}
}

/************************/
/**     PID��ʼ��      **/
/************************/
void PIDInit (struct PID *pp) 
{ 
	memset (pp,0,sizeof(struct PID)); 			//�ò���0��ʼ��pp
} 

/************************/
/**      PID����       **/
/************************/
unsigned int PIDCalc(struct PID *pp, unsigned int NextPoint )
{ 
	unsigned int dError,Error; 
	Error = pp->SetPoint - NextPoint; 			//ƫ�� 
	pp->SumError += Error; 									//���� 
	dError = pp->LastError - pp->PrevError; //��ǰ΢�� 
	pp->PrevError = pp->LastError; 
	pp->LastError = Error; 
	return (pp->Proportion * Error					//����
	+ pp->Integral * pp->SumError  					//������
	+ pp->Derivative * dError); 						//΢���� 
} 

/*******************************************************/ 
/**               �¶ȱȽϴ����ӳ���                  **/
/*******************************************************/

void compare_temper(void) 
{ 

	unsigned char i; 
	temper = get_temper()/10;
	if(set_temper_min>temper) 								//�����¶ȴ��ڵ�ǰ�¶�
	{
		ledred=0;
		ledgreen=1;
		if(set_temper_min-temper>1) 						//�¶����1������
		{	 
			high_time=100;
			low_time=0;
		} 
		else 																		//�����¶Ȳ����ڵ�ǰ�¶�
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
	else if(set_temper_max<=temper) 					//�����¶Ȳ����ڵ�ǰ�¶�
	{ 
		ledred=1;
		ledgreen=0;
		if(temper-set_temper_max>0) 						//�¶����0������
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
	if(set_temper_min>=temper) 								//�����¶ȴ��ڵ�ǰ�¶�
	{
		ledred=0;
		ledgreen=1;
	} 
	else if(set_temper_max<temper) 					//�����¶Ȳ����ڵ�ǰ�¶�
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
/**           T0�жϷ����ӳ������ڿ��Ƶ�ƽ�ķ�ת ,40us*100=4ms����              **/
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


/***********������**********/
void main(void)
{
	/**unsigned char i;**/
	init();//LED��ʼ��
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
		keyscan(); 										//����ɨ��
		dis_temp(get_temper());				//��ʾ�¶�ֵ
		compare_temper(); 						//�Ƚ��¶�
	}
}