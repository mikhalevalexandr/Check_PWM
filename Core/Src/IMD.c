#include "IMD.h"
#include "main.h"
#include "math.h"
extern TIM_HandleTypeDef htim2;
uint32_t BuffPeriodIMD[20];
uint32_t BuffDutyCycleIMD[20];
double frequency=0;
double DutyCycleIMD;
double InsulationResistance=0;// сопротивление изоляции в МОм
uint32_t PeriodMedian;
uint32_t DutyCycleMedian;
//double huila;
uint32_t TIM_FOR_IMD_PWM_CLOCK=0;
char IMD_Condititon[100];
uint32_t MedianArray(uint32_t *arr, size_t size) {
    // Находим min и max массива,
    // чтобы получить наибольшую разность;
		uint32_t median = 0;
    uint32_t max = -1;
    uint32_t min = arr[0];
    uint32_t left_sum = 0;
    uint32_t right_sum = 0;
    for (size_t i = 1;i < size - 1;i++) {  // Минимальная разность		 
		left_sum = 0;
		right_sum = 0;
		for (size_t j = 0;j < i;j++)
		    left_sum += arr[j];
		for (size_t k = i + 1;k < size;k++)
		    right_sum += arr[k];
		if (abs(left_sum - right_sum) >= max)
		    max = abs(left_sum - right_sum);  
    } 
    for (size_t i = 1;i < size - 1;i++) {
	left_sum = 0;
	right_sum = 0;
	for (size_t j = 0;j < i;j++)
	    left_sum += arr[j];
	for (size_t k = i + 1;k < size;k++)
	    right_sum += arr[k];
	if (abs(left_sum - right_sum) <= max)
	    max = abs(left_sum - right_sum);
    }
    for (size_t i = 1;i < size - 1;i++) {
	left_sum = 0;
	right_sum = 0;
	for (size_t j = 0;j < i;j++)
	    left_sum += arr[j];
	for (size_t k = i + 1;k < size;k++)
	    right_sum += arr[k];
	if (abs(left_sum - right_sum) == max)
	    median = i;
    }
	return arr[median];
}
uint32_t GETPCLK2TIM(void) //  Получение значения тактирования на таймере 1
{
  /* Get PCLK2 frequency */
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
 
  /* Get PCLK2 prescaler */
  if((RCC->CFGR & RCC_CFGR_PPRE2) == 0)
  {
    /* PCLK2 prescaler equal to 1 => TIMCLK = PCLK2 */
    return (pclk2);
  }
  else
  {
    /* PCLK2 prescaler different from 1 => TIMCLK = 2 * PCLK2 */
    return(2 * pclk2);
  }
	
}
uint32_t GETPCLK1TIM(void) //  Получение значения тактирования на таймере 1
{
  /* Get PCLK2 frequency */
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
 
  /* Get PCLK2 prescaler */
  if((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
  {
    /* PCLK2 prescaler equal to 1 => TIMCLK = PCLK2 */
    return (pclk1);
  }
  else
  {
    /* PCLK2 prescaler different from 1 => TIMCLK = 2 * PCLK2 */
    return(2 * pclk1);
  }
	
}
void InsulationConditionFromIMD (uint32_t* BuffPeriod, uint32_t* BuffDutyCycle)
{
	size_t PeriodSize=sizeof(BuffPeriod)/sizeof(uint32_t);
	size_t DutyCycleSize=sizeof(BuffPeriod)/sizeof(uint32_t);
	PeriodMedian=MedianArray(BuffPeriod, PeriodSize);
	DutyCycleMedian=MedianArray(BuffDutyCycle, DutyCycleSize);
	TIM_FOR_IMD_PWM_CLOCK=GETPCLK1TIM();
	uint32_t eps=2;// погрешность измерения частоты
	int length=0;//длина сообщения
	//uint32_t frequency=0;
	//float DutyCycleIMD=0;
	//float InsulationResistance=0;// сопротивление изоляции в МОм
	
//	frequency=1*( GETPCLK2TIM()%(htim2.Init.Prescaler+1))%periodCall;// - Гц, так период в мкс, чтобы его перевести в секунды его \
																																нужно поделить на TIM1_CLOCK%(htim2.Init.Prescaler+1) :\
																																(1/(period/TIM1_CLOCK%(htim2.Init.Prescaler+1))=\
																																=1*TIM1_CLOCK%(htim2.Init.Prescaler+1)/period), чтобы получить частоту в Гц  
	frequency=((double)TIM_FOR_IMD_PWM_CLOCK/(double)(htim2.Init.Prescaler+1))/(double)PeriodMedian;
	DutyCycleIMD=(double)DutyCycleMedian/(double)PeriodMedian;
	 //huila=fabs(frequency-50);
	if (fabs(frequency-50)<=eps)
	{
		if (DutyCycleIMD<=0.55 && DutyCycleIMD>=0.45)
		{
			InsulationResistance=0; //Но на самом деле неизвестно, так как состояние Connection fault earth
			length=sprintf (IMD_Condititon, "Fault detected on the earth connection");
		}
	}
	else if (fabs(frequency-40)<=eps)
	{
		if (DutyCycleIMD<=0.55 && DutyCycleIMD>=0.45)
		{
			InsulationResistance=0; //Но на самом деле неизвестно, так как состояние Devise error
			length=sprintf (IMD_Condititon, "Devise error");
		}
	}
	else if (fabs(frequency-30)<=eps)
	{
		if (DutyCycleIMD<=0.125 && DutyCycleIMD>=0.045)
		{
			InsulationResistance=0; //Но на самом деле неизвестно, так как состояние Devise error
			length=sprintf (IMD_Condititon, "Speed start measurement: good");
		}
		if (DutyCycleIMD>=0.875 && DutyCycleIMD<=0.975)
		{
			InsulationResistance=0; //Но на самом деле неизвестно, так как состояние Devise error
			length=sprintf (IMD_Condititon, "Speed start measurement: bad");
		}
		
	}
	else if (fabs(frequency-20)<=eps)
	{
		if (DutyCycleIMD<=0.975 && DutyCycleIMD>=0.045)
		{
			InsulationResistance=0; //Но на самом деле неизвестно, так как состояние Devise error
			length=sprintf (IMD_Condititon, "Undervoltage condition: <350V");
		}
	}
	else if (fabs(frequency-10)<=eps)
	{
		length=sprintf (IMD_Condititon, "Insulation measurement:");
		if (DutyCycleIMD<=0.05)
		{
			InsulationResistance=50; // МОм и больше
		}
		else if (DutyCycleIMD>0.05)
		{
			InsulationResistance=((90*1200)/(DutyCycleIMD*100-5)-1200)/1000; //МОм
		}
		else if (DutyCycleIMD>0.05)
		{
			InsulationResistance=0; //МОм
		}
	}
	else 
	{
		length=sprintf (IMD_Condititon, "WTF");
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== GPIO_PIN_0) {
		//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
		//*huila=(HAL_RCC_GetPCLK1Freq()) ;
		//frequency=*huila;
		//frequency=(HAL_RCC_GetPCLK1Freq()) ;
		InsulationConditionFromIMD( BuffPeriodIMD, BuffDutyCycleIMD);
 } 
	//else{

//    __NOP();

//  }
}
