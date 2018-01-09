#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "string.h"
#include "driverlib/adc.h"
#include "uartstdio.h"
#include "lm4f120h5qr.h"
#include "driverlib/qei.h"


#define sol_kat 0.35
#define sag_kat 0.35
#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401
#define sensor1  GPIO_PIN_5 
#define sensor2  GPIO_PIN_3
#define sensor3  GPIO_PIN_2
#define sensor4  GPIO_PIN_2
#define sensor5	 GPIO_PIN_0
#define sensor6  GPIO_PIN_1
#define sensor7  GPIO_PIN_4
#define sensor8  GPIO_PIN_3
#define sagsay1											GPIO_PIN_4//Birinci motor
#define sagsay2											GPIO_PIN_2
#define solsay1											GPIO_PIN_6//ikinci motor
#define solsay2											GPIO_PIN_7
#define GPIO_PF2_T1CCP0         0x00050807
#define GPIO_PF3_T1CCP1         0x00050C07
#define GPIO_PF1_T0CCP1         0x00050407
#define GPIO_PB6_T0CCP0         0x00011807
#define GPIO_PB7_T0CCP1         0x00011C07
#define GPIO_PB0_T2CCP0         0x00010007
#define GPIO_PB1_T2CCP1         0x00010407
#define GPIO_PD7_CCP1           0x00031C03
#define GPIO_PB2_CCP3           0x00010804
#define GPIO_O_LOCK             0x00000520  // GPIO Lock
#define GPIO_LOCK_KEY_DD        0x4C4F434B 
#define GPIO_O_CR               0x00000524
#define GPIO_PC4_WT0CCP0        0x00021007
#define GPIO_PC5_WT0CCP1        0x00021407
#define GPIO_PC6_WT1CCP0        0x00021807
#define GPIO_PC7_WT1CCP1        0x00021C07

/////////////////////////////////////////
long sagMotorHizi=46000,solMotorHizi=46000;

///////////////////////////////////////////////
unsigned long uzak,ulADC0_Value[5];
signed int f1,f2,f3;
unsigned long  dutyCycle3, dutyCycle4,hiz,hiz1;
unsigned char a[10],i,i1=0,u,p,k=0,run=0,run1,a1=0,b1=0;
unsigned long int t,t1,seri,u1,Period,i_m,sayac1=0,sayac2=0,kont=0,oku_say,say1=0,say2=0;
unsigned char ch[20];

unsigned long ulADC0_Value[5],sag_motor, sol_motor,ulPeriod=64000;
unsigned long ulADC1_Value[5];
void __error__(char *pcFilename, unsigned long ulLine)
{
}

void delay (unsigned long int id)
{
		while(id--);
}



void IntGPIOF(void)
{
	if (GPIOPinIntStatus(GPIO_PORTF_BASE, GPIO_PIN_0) & GPIO_PIN_0)
	{
		GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
		
			run1=0;
		
 
	}else{
	    GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
	
			run1=1;
				
	}
}

void dur()
{

		TimerMatchSet(WTIMER0_BASE, TIMER_A, ulPeriod-2); // Timer 0 Match set
		TimerMatchSet(WTIMER1_BASE, TIMER_B, ulPeriod-2 ); // Timer 0 Match et
	  TimerMatchSet(WTIMER0_BASE, TIMER_B, ulPeriod-2); // Timer 0 Match set
		TimerMatchSet(WTIMER1_BASE, TIMER_A, ulPeriod-2 ); // Timer 0 Match et
	
}
void sag()
{
		sol_motor = (ulPeriod-1)*(0.01);
		sag_motor = (ulPeriod-1)*(0.99);
		TimerMatchSet(WTIMER0_BASE, TIMER_A,  sag_motor); // Timer 0 Match set
		TimerMatchSet(WTIMER1_BASE, TIMER_B, sol_motor); // Timer 0 Match et
	  TimerMatchSet(WTIMER0_BASE, TIMER_B,  sag_motor); // Timer 0 Match set
		TimerMatchSet(WTIMER1_BASE, TIMER_A, sol_motor); // Timer 0 Match et
		TimerEnable(WTIMER0_BASE, TIMER_BOTH); 
		TimerEnable(WTIMER1_BASE, TIMER_BOTH); 
}
void dur2()
{
		sol_motor = (ulPeriod-1)*sol_kat;
		sag_motor = (ulPeriod-1)*sag_kat;
		TimerMatchSet(WTIMER0_BASE, TIMER_A,  sag_motor); // Timer 0 Match set
		TimerMatchSet(WTIMER1_BASE, TIMER_B, sol_motor); // Timer 0 Match et
	  TimerMatchSet(WTIMER0_BASE, TIMER_B,  sag_motor); // Timer 0 Match set
		TimerMatchSet(WTIMER1_BASE, TIMER_A, sol_motor); // Timer 0 Match et
		TimerEnable(WTIMER0_BASE, TIMER_BOTH); 
		TimerEnable(WTIMER1_BASE, TIMER_BOTH); 
	
}
void ileri(long a,long b)
{	
	if(a>ulPeriod-2)
		a=ulPeriod-2;
		if(b>ulPeriod-2)
		b=ulPeriod-2;
	
	TimerMatchSet(WTIMER0_BASE, TIMER_A, a); // Timer 0 Match set
  TimerMatchSet(WTIMER0_BASE, TIMER_B, ulPeriod-2); // Timer 0 Match set
	TimerMatchSet(WTIMER1_BASE, TIMER_A, b); // Timer 0 Match set
	TimerMatchSet(WTIMER1_BASE, TIMER_B, ulPeriod-2); // Timer 0 Match et
	TimerEnable(WTIMER0_BASE, TIMER_A);
	TimerEnable(WTIMER0_BASE, TIMER_B);
	TimerEnable(WTIMER1_BASE, TIMER_A);
	TimerEnable(WTIMER1_BASE, TIMER_B);
	
}









// void hiz_olc1(void)
// {

//   hiz1=SysCtlClockGet()-TimerValueGet(TIMER1_BASE,TIMER_A);
// 	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet());

// }

// void hizolc(void)
// {

//   hiz=SysCtlClockGet()-TimerValueGet(TIMER0_BASE,TIMER_A);
// 	  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());

// }

void IntGPIOA(void)
{
	
	if (GPIOPinIntStatus(GPIO_PORTA_BASE, sagsay1) & sagsay1)
		{
				GPIOPinIntClear(GPIO_PORTA_BASE, sagsay1);
				if(!GPIOPinRead(GPIO_PORTA_BASE,sagsay2))
				{
						sayac1--;		
			

					
				}
				else
				{
						sayac1++;
					//	hizolc();

				}
				
		}
	 if(GPIOPinIntStatus(GPIO_PORTA_BASE, sagsay2) & sagsay2 )
	 {
			GPIOPinIntClear(GPIO_PORTA_BASE, sagsay2);
			if(!GPIOPinRead(GPIO_PORTA_BASE,sagsay1))
			{
			sayac1++;		
	
			}
			else
			{
					sayac1--;
	

			}
	 }
	if (GPIOPinIntStatus(GPIO_PORTA_BASE, solsay1) & solsay1)
	{
		GPIOPinIntClear(GPIO_PORTA_BASE, solsay1);
		if(!GPIOPinRead(GPIO_PORTA_BASE,solsay2))
			{
			sayac2--;
		
		   }
		else
			{
	 		sayac2++;
	  	//hiz_olc1();
		  }
	}
	else if(GPIOPinIntStatus(GPIO_PORTA_BASE, solsay2) & solsay2 )
	{
		GPIOPinIntClear(GPIO_PORTA_BASE, solsay2);
		if(!GPIOPinRead(GPIO_PORTA_BASE,solsay1))
			{
			sayac2++;	
   		
		 }
		else
		{
			sayac2--;
		
		}
	}
	
		UARTprintf("%d \n",   hiz);

	
	
}
// void HizAyarla (long a,long b)
//  
// {	

// 	
// 	if(hiz>a)
// 	{
// 		sagMotorHizi=sagMotorHizi-2500;
// 	}
// 	if(hiz<a)
// 	{
// 		sagMotorHizi=2500+sagMotorHizi;
// 	}
// 	if(hiz1>b)
// 	{
// 		solMotorHizi=solMotorHizi-2000;
// 	}
// 	if(hiz1<b)
// 	{
// 		solMotorHizi=2000+solMotorHizi;
// 	}
// 	
//    if(solMotorHizi>54000)
// 		solMotorHizi=54000;
// 	if(solMotorHizi<35000)
// 		solMotorHizi=35000;
// 	
// 		if(sagMotorHizi>54000)
// 		sagMotorHizi=54000;
// 	if(sagMotorHizi<35000)
// 		sagMotorHizi=35000;
// 	

// 	ileri(sagMotorHizi,solMotorHizi);

// 	
// }

void duz_ileri2(void)
{
	if( (!GPIOPinRead(GPIO_PORTE_BASE,sensor1)))
			{
				
				sagMotorHizi-=6000;
			  solMotorHizi+=4000;
				
			}
			 else  if((!GPIOPinRead(GPIO_PORTF_BASE,sensor2)))
			{
				
					sagMotorHizi-=5000;
			    solMotorHizi+=3000;
				
			}
			 else if( (!GPIOPinRead(GPIO_PORTF_BASE,sensor3)))
			{
				
			   sagMotorHizi-=4000;
			   solMotorHizi+=2000;
				
			}
			
			else if( (!GPIOPinRead(GPIO_PORTB_BASE,sensor6)) )
			{
		
			          sagMotorHizi+=2000;
				        solMotorHizi-=2000;
			   
				
			}
			else if( (!GPIOPinRead(GPIO_PORTB_BASE,sensor7)))
			{
				
							   sagMotorHizi+=3000;
								solMotorHizi-=3000;
			          
				
			}
			else if( (!GPIOPinRead(GPIO_PORTE_BASE,sensor8)))
			{
				
						     sagMotorHizi+=4000;
									solMotorHizi-=4000;
			       
				
			}
			else if( (!GPIOPinRead(GPIO_PORTB_BASE,sensor4))&&(!GPIOPinRead(GPIO_PORTE_BASE,sensor5)) )
			{
			
			sagMotorHizi=40000;
			solMotorHizi=40000;

			}
	if (sagMotorHizi < 35000 ) sagMotorHizi = 35000; 
  if (solMotorHizi < 35000 ) solMotorHizi = 35000;
	if (sagMotorHizi >= 50000) sagMotorHizi = 50000; 
  if (solMotorHizi >= 50000) solMotorHizi = 50000; 
	ileri(sagMotorHizi,solMotorHizi);
}


// void duz_ileri1(void)
// {
// 		if( (!GPIOPinRead(GPIO_PORTB_BASE,sensor4))|| (!GPIOPinRead(GPIO_PORTE_BASE,sensor5)))
// 			{
// 		    HizAyarla(1650000,1650000);

// 			}
// 	      else if( (!GPIOPinRead(GPIO_PORTE_BASE,sensor1)))
// 			{
// 				
// 				HizAyarla(1250000,2000000);
// 				
// 			}
// 			 else  if((!GPIOPinRead(GPIO_PORTF_BASE,sensor2)))
// 			{
// 					HizAyarla(1350000,1900000);
// 					
// 				
// 			}
// 			 else if( (!GPIOPinRead(GPIO_PORTF_BASE,sensor3)))
// 			{
// 					HizAyarla(1400000,1800000);
// 			   
// 				
// 			}
// 			
// 			else if( (!GPIOPinRead(GPIO_PORTB_BASE,sensor6)) )
// 			{
// 		
// 			        
// 			   	HizAyarla(2200000,1500000);
// 				
// 			}
// 			else if( (!GPIOPinRead(GPIO_PORTB_BASE,sensor7)))
// 			{
// 				
// 							
// 			          HizAyarla(2300000,1400000);
// 				
// 			}
// 			else if( (!GPIOPinRead(GPIO_PORTE_BASE,sensor8)))
// 			{
// 				
// 						
// 			       HizAyarla(2425000,1300000);
// 				
// 			}
// 	
//  
// 	

// }








void  sonic(){
			
		ADCProcessorTrigger(ADC0_BASE, 1);
	
while(!ADCIntStatus(ADC0_BASE, 1, false))
{
}
ADCIntClear(ADC0_BASE, 1);

ADCSequenceDataGet(ADC0_BASE, 1, ulADC0_Value);

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0,GPIO_PIN_0 ); //ir ledi yakar
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6,GPIO_PIN_6 ); //ir ledi yakar
	SysCtlDelay(1500); //transitörün iletime geçmesi için bekle

	ADCProcessorTrigger(ADC0_BASE, 1); //ADC pil ve iki sensöredeki gerilimleri okur
while(!ADCIntStatus(ADC0_BASE, 1, false))//ADC' nin gerilimleri okumumasini bekliyor
{
}
ADCIntClear(ADC0_BASE, 1);

GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0,0 );//ir ledler sonük
GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6,0 );//ir ledler sonük
ADCSequenceDataGet(ADC0_BASE, 1, ulADC1_Value);//sequence_1 e yazilan sonuclari okur
UARTprintf("%d   %d    %d\n", ulADC0_Value[0],ulADC0_Value[1],ulADC0_Value[2]);

f1=ulADC0_Value[0]-ulADC1_Value[0];
f2=ulADC0_Value[1]-ulADC1_Value[1];
f3=ulADC0_Value[2];
		
	
}
void Timer2IntHandler(void)
{
        TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
sonic();
		UARTprintf("pil %d   dol %d    %d\n", ulADC0_Value[0],ulADC0_Value[1],ulADC0_Value[2]);
}
		
void Timer3IntHandler(void)
{
        TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        sonic();
				//UARTprintf("pil %d   dol %d    %d\n", ulADC0_Value[0],ulADC0_Value[1],ulADC0_Value[2]);
}

int main(void)
{
	
    ulPeriod = 6400000;
  SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);	
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;//sw2 serbest
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);//sw1 basla sw2 dur
	
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,sagsay1|sagsay2|solsay1|solsay2);
	
	GPIOPadConfigSet(GPIO_PORTA_BASE, sagsay1|sagsay2|solsay1|solsay2, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
  GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_FALLING_EDGE);
  GPIOIntTypeSet(GPIO_PORTA_BASE, sagsay1|sagsay2|solsay1|solsay2, GPIO_RISING_EDGE);

	
	GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
	GPIOPinIntEnable(GPIO_PORTA_BASE, sagsay1|sagsay2|solsay1|solsay2);

//	IntEnable(INT_GPIOF);
	 IntEnable(INT_GPIOA);
//IntEnable(INT_GPIOB);

	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

	GPIOPinConfigure(GPIO_PC4_WT0CCP0); 
	GPIOPinConfigure(GPIO_PC5_WT0CCP1); 
	GPIOPinConfigure(GPIO_PC6_WT1CCP0);
	GPIOPinConfigure(GPIO_PC7_WT1CCP1);

	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4 ); 
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5 ); 
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6 ); 
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_7 ); 

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

TimerConfigure(TIMER2_BASE, TIMER_CFG_32_BIT_PER);
	
	TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);	
	
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet());
	



	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/10);
  TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3); 
TimerConfigure(TIMER3_BASE, TIMER_CFG_32_BIT_PER); 
 TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet()/50); //saniyede 20 kez sensorleri oku
 
  TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

 IntEnable(INT_TIMER3A);
 
 
	IntEnable(INT_TIMER2A);
	
	TimerConfigure(WTIMER0_BASE, (TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM));
	TimerConfigure(WTIMER1_BASE, (TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM));
	
	TimerControlLevel(WTIMER0_BASE, TIMER_BOTH, 0); 
	TimerControlLevel(WTIMER1_BASE, TIMER_BOTH, 0);
	
	ulPeriod = 64000;

	TimerLoadSet(WTIMER0_BASE, TIMER_A, ulPeriod -1);
	TimerLoadSet(WTIMER0_BASE, TIMER_B, ulPeriod -1);
	TimerLoadSet(WTIMER1_BASE, TIMER_A, ulPeriod -1);
	TimerLoadSet(WTIMER1_BASE, TIMER_B, ulPeriod -1);
			
	TimerMatchSet(WTIMER0_BASE, TIMER_A,  ulPeriod-2);
	TimerMatchSet(WTIMER0_BASE, TIMER_B,  ulPeriod-2);
	TimerMatchSet(WTIMER1_BASE, TIMER_A,  ulPeriod-2);
	TimerMatchSet(WTIMER1_BASE, TIMER_B,  ulPeriod-2);
	
	TimerEnable(WTIMER0_BASE, TIMER_BOTH); 
	TimerEnable(WTIMER1_BASE, TIMER_BOTH); 

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));  
  IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
  UARTStdioInit(0);
  IntMasterEnable();
	UARTprintf("deneme\n");

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,GPIO_PIN_1);

	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,sensor4|sensor6|sensor7);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE,sensor1|sensor5|sensor8);	
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,sensor2|sensor3);	
	
		
	GPIOPadConfigSet(GPIO_PORTB_BASE,sensor4|sensor6|sensor7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTE_BASE,sensor1|sensor5|sensor8,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTF_BASE,sensor2|sensor3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
	
	//ADC
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
SysCtlADCSpeedSet(SYSCTL_ADCSPEED_1MSPS);
GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);
GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);

ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH7);
ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH9 | ADC_CTL_IE |ADC_CTL_END);

ADCSequenceEnable(ADC0_BASE, 1);
ADCIntClear(ADC0_BASE, 1);
	
	//ADC
	
	
//   hiz=SysCtlClockGet()-TimerValueGet(TIMER0_BASE,TIMER_A);
// 	  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
// 		
// 		hiz1=SysCtlClockGet()-TimerValueGet(TIMER1_BASE,TIMER_A);
// 	  TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet());

//max 380 400 düz gitme için
	//TimerMatchSet(WTIMER0_BASE, TIMER_A, 45000); // Timer 0 Match set
//	TimerMatchSet(WTIMER1_BASE, TIMER_A, 45000); // Timer 0 Match set


// while(1)
// 	if( (!GPIOPinRead(GPIO_PORTE_BASE,sensor6)))
// 			
// 	ileri(35000,35000);
// else dur();
// oku_say=0;
       
        


while(1)
{
	sonic();
// 	duz_ileri2();
	ileri(100,100);
if(f3>1800){
	 sag();
oku_say=0;
while(oku_say<3) ;
}
}
// 	ileri(380,380);
//yavas ilerisi 420 440


}