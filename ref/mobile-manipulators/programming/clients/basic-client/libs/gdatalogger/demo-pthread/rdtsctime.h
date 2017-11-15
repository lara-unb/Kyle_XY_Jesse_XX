typedef struct {
	unsigned int l;
	unsigned int h;
} CLOCK_COUNTER,*PCLOCK_COUNTER;

double __CpuFreq;

/*********************************************************************************************
**********************************************************************************************
** RDTSC_SetCPUClock: 
**		Set CPU clock frequency. Must be used before any call to RDTSC_ComputeTimeInterval.
**********************************************************************************************
*********************************************************************************************/

#define RDTSC_SetCPUClock(Freq)	__CpuFreq = Freq

/*********************************************************************************************
**********************************************************************************************
** RDTSC_GetClock: 
**		Recover CPU clock counter. 
**********************************************************************************************
*********************************************************************************************/

#define RDTSC_GetClock(Clock)	asm("rdtsc");	asm("mov %%eax, %0": "=r"(Clock.l));	asm("mov %%edx, %0": "=r"(Clock.h));


/*********************************************************************************************
**********************************************************************************************
** RDTSC_ComputeTimeInterval: 
**		Compute the time interval elapsed between pckStart and pckStop in seconds. 
**		Returns -1.0 if pckStop < pckStart. 
**********************************************************************************************
*********************************************************************************************/

__inline double RDTSC_ComputeTimeInterval(PCLOCK_COUNTER pckStart, PCLOCK_COUNTER pckStop)
{
	unsigned long long ckInterval;
	unsigned long long ckStart,ckStop;

	ckStart  = pckStart->h;
	ckStart  = ckStart << 32;
	ckStart += pckStart->l;

	ckStop  = pckStop->h;
	ckStop  = ckStop << 32;
	ckStop += pckStop->l;

	if (ckStop < ckStart){
		//return (-1.0);
	}

	ckInterval = ckStop-ckStart;

	return( ((double)((long long )(ckInterval)))/__CpuFreq );

}

