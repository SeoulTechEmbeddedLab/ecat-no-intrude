/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  to test EtherCAT master protocol using the IgH EtherCAT master userspace library.	
 *  
 *  
 *  IgH EtherCAT master library for Linux is found at the following URL: 
 *  <http://www.etherlab.org/en/ethercat>
 *
 *
 *
 *
 *  2015 Raimarius Delgado
*/
/****************************************************************************/
#include "main.h"
/****************************************************************************/
int  XenoInit(void);
void XenoQuit(void);
void DoInput();
void SignalHandler(int signum);
void PrintEval(int BufPrd[], int BufExe[], int BufJit[], int ArraySize);
void FilePrintEval(char *FileName,int BufPrd[], int BufExe[], int BufJit[], int ArraySize);
/****************************************************************************/
void EcatCtrlTask(void *arg){
	int iSlaveCnt;
	int iTaskTick = 0;
#ifdef MEASURE_TIMING
	RtmEcatPeriodEnd = rt_timer_read();
#endif
	while (1){

	rt_task_wait_period(NULL);
#ifdef MEASURE_TIMING
	RtmEcatPeriodStart = rt_timer_read();
#endif
	/* Receive Process Data */
	EcatReceiveProcessDomain();
    	/* check process data state (optional) */
	EcatState = EcatStatusCheck(); 

	/*
	 * Do reading of the current process data from here
	 * before processing and sending to Tx
	 */ 
	
	/*Read EtherCAT Datagram*/
	if(EcatState.master_state == OP && EcatState.slave_state == OP){
		
		for(iSlaveCnt=0; iSlaveCnt < SANYODENKI_SLAVENUM; ++iSlaveCnt){
			
			sanyoStatusWord[iSlaveCnt] = sanyoGetStatusWordN(iSlaveCnt);
			sanyoActualVelR[iSlaveCnt] = sanyoGetActualVelocityN(iSlaveCnt);

			if(sanyoPrevStatusWord[iSlaveCnt] == sanyoStatusWord[iSlaveCnt]){
				continue;
			}
			switch(sanyoStatusWord[iSlaveCnt]){	
				case SANYO_SWITCH_ON_DISABLED:
					sanyoReady(iSlaveCnt);
					break;
				case SANYO_READY_TO_SWITCH_ON:
					sanyoSwitchOn(iSlaveCnt);
					break;
				case SANYO_SWITCH_ON_ENABLED:
					sanyoServoOn(iSlaveCnt);
					break;
				case SANYO_OPERATION_ENABLED:
					sanyoServoOp = RUN;
					break;
				case SANYO_FAULT:
					sanyoFaultReset(iSlaveCnt);
					break;
				default:
					break;
			}
			sanyoPrevStatusWord[iSlaveCnt] = sanyoStatusWord[iSlaveCnt]; 
		}
	}
	switch(sanyoServoOp){
	
		case RUN:
#ifdef PERF_EVAL	   
			if (bTimingFlag == FALSE) bTimingFlag = TRUE;		
#endif //PERF_EVAL
			sanyoSetVelocity(_ALLNODE,500);
			break;
		case STOP:
			sanyoShutDown(_ALLNODE);
			bQuitFlag = TRUE;
			break;
		default:
			break;
	}
	/* write application time to master */
	RtmEcatMasterAppTime = rt_timer_read();
	EcatWriteAppTimeToMaster((uint64_t)RtmEcatMasterAppTime);
	/* send process data */
	EcatSendProcessDomain();

#ifdef MEASURE_TIMING
	RtmEcatExecTime = rt_timer_read();
	EcatPeriod 	 = ((int)RtmEcatPeriodStart - (int)RtmEcatPeriodEnd);
	EcatExecution = ((int)RtmEcatExecTime - (int)RtmEcatPeriodStart);
	EcatJitter	 = MathAbsValI(EcatPeriod - (int)ECATCTRL_TASK_PERIOD);
#endif
 
#if 0 //  0 not visible : 1 visible
	if (!(iTaskTick % FREQ_PER_SEC(ECATCTRL_TASK_PERIOD))){
		/*Do every 1 second */
		rt_printf("Task Duration: %d s \n", iTaskTick/FREQ_PER_SEC(ECATCTRL_TASK_PERIOD));
	}
#endif


#ifdef PERF_EVAL	   
	if(bTimingFlag == TRUE) {
		BufEcatPeriodTime[iBufEcatDataCnt] 	=   EcatPeriod;
		BufEcatExecTime[iBufEcatDataCnt]  	=   EcatExecution;
		BufEcatJitter[iBufEcatDataCnt]  	=   EcatJitter;
		++iBufEcatDataCnt;

		if(iBufEcatDataCnt == BUF_SIZE){
			bTimingFlag = FALSE;
			sanyoServoOp = STOP;
			}
	}
#endif //PERF_EVAL


#ifdef MEASURE_TIMING	    
   	   RtmEcatPeriodEnd = RtmEcatPeriodStart;
#endif //MEASURE_TIMING
	   iTaskTick++;
    }
}
/****************************************************************************/
int main(int argc, char **argv){
	int ret = 0;

	/* Interrupt Handler "ctrl+c"  */
	signal(SIGTERM, SignalHandler);
        signal(SIGINT, SignalHandler);	 

	/* EtherCAT Init */
   	if ((ret = EcatInit(ECATCTRL_TASK_PERIOD,SANYO_CYCLIC_VELOCITY))!=0){
		fprintf(stderr, 
			"Failed to Initiate IgH EtherCAT Master!\n");
		return ret;
	}
	
	mlockall(MCL_CURRENT|MCL_FUTURE); //Lock Memory to Avoid Memory Swapping

	/* RT-task */
	if ((ret = XenoInit())!=0){
		fprintf(stderr, 
			"Failed to Initiate Xenomai Services!\n");
		return ret;
	}

	while (1) {
		usleep(1);
		if (bQuitFlag) break;
	}

#ifdef PERF_EVAL
#ifdef PRINT_TIMING

	FilePrintEval((char *)PRINT_FILENAME,BufEcatPeriodTime,
			BufEcatExecTime,BufEcatJitter,iBufEcatDataCnt);

#endif // PRINT_TIMING
	
	PrintEval(BufEcatPeriodTime,BufEcatExecTime,BufEcatJitter,iBufEcatDataCnt);

#endif //PERF_EVAL

	XenoQuit();
	EcatQuit();

return ret;
}

/****************************************************************************/
void SignalHandler(int signum){
		bQuitFlag = TRUE;
}

/****************************************************************************/

int XenoInit(void){

	rt_print_auto_init(1); //RTDK

	printf("Creating Xenomai Realtime Task(s)...");
	if(rt_task_create(&TskEcatCtrl,"EtherCAT Control", 0, 
			  ECATCTRL_TASK_PRIORITY,ECATCTRL_TASK_MODE)){
	
      		fprintf(stderr, "Failed to create Ecat Control Task\n");
		return _EMBD_RET_ERR_;
	}
	printf("OK!\n");


	printf("Making Realtime Task(s) Periodic...");
	if(rt_task_set_periodic(&TskEcatCtrl, TM_NOW,rt_timer_ns2ticks(ECATCTRL_TASK_PERIOD))){
	
		fprintf(stderr, "Failed to Make Ecat Control Task Periodic\n");
		return _EMBD_RET_ERR_;
	}
	printf("OK!\n");

	printf("Starting Xenomai Realtime Task(s)...");
	if(rt_task_start(&TskEcatCtrl, &EcatCtrlTask, NULL)){
		fprintf(stderr, "Failed to start Ecat Control Task\n");
		return _EMBD_RET_ERR_;
	}
	printf("OK!\n");

	return _EMBD_RET_SCC_;
}

/****************************************************************************/

void XenoQuit(void){
	rt_task_suspend(&TskEcatCtrl);
	
	rt_task_delete(&TskEcatCtrl);
	printf("\033[%dm%s\033[0m",95,"Xenomai Task(s) Deleted!\n");
}

/****************************************************************************/

void PrintEval(int BufPrd[], int BufExe[], int BufJit[], int ArraySize){

/* MATH_STATS: Simple Statistical Analysis (libs/embedded/embdMATH.h)
 * 	float ave;
 * 	float max;
 * 	float min;
 * 	float std; */
	MATH_STATS EcatPeriodStat, EcatExecStat, EcatJitterStat;

	EcatPeriodStat = GetStatistics(BufPrd, ArraySize,SCALE_1M);  
	printf("\n[Period] Max: %.6f Min: %.6f Ave: %.6f St. D: %.6f\n", EcatPeriodStat.max,
			EcatPeriodStat.min,
			EcatPeriodStat.ave,
			EcatPeriodStat.std);
	EcatExecStat = GetStatistics(BufExe, ArraySize,SCALE_1K);  
	printf("[Exec]	 Max: %.3f Min: %.3f Ave: %.3f St. D: %.3f\n", EcatExecStat.max,
			EcatExecStat.min,
			EcatExecStat.ave,
			EcatExecStat.std);
	EcatJitterStat = GetStatistics(BufJit, ArraySize,SCALE_1K);  
	printf("[Jitter] Max: %.3f Min: %.3f Ave: %.3f St. D: %.3f\n", EcatJitterStat.max,
			EcatJitterStat.min,
			EcatJitterStat.ave,
			EcatJitterStat.std);
}

/****************************************************************************/

void FilePrintEval(char *FileName,int BufPrd[], int BufExe[], int BufJit[], int ArraySize){

	FILE *FileEcatTiming;
	int iCnt;

	FileEcatTiming = fopen(FileName, "w");

	for(iCnt=0; iCnt < ArraySize; ++iCnt){
		fprintf(FileEcatTiming,"%d.%06d,%d.%03d,%d.%03d\n",
				BufPrd[iCnt]/SCALE_1M,
				BufPrd[iCnt]%SCALE_1M,
				BufExe[iCnt]/SCALE_1K,
				BufExe[iCnt]%SCALE_1K,
				BufJit[iCnt]/SCALE_1K,
				BufJit[iCnt]%SCALE_1K);
	}
	fclose(FileEcatTiming);
}

/****************************************************************************/
