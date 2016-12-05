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
			if (!(timingFlag)) timingFlag = 1;		
			sanyoSetVelocity(_ALLNODE,500);
			break;
		case STOP:
			sanyoShutDown(_ALLNODE);
			quitFlag = 1;
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
	RtmEcatExecTime    = rt_timer_read();

	temp_EcatPeriod	   = ((int)RtmEcatPeriodStart - (int)RtmEcatPeriodEnd);
	temp_EcatExecution = ((int)RtmEcatExecTime - (int)RtmEcatPeriodStart);
	temp_EcatJitter	   = MathAbsValI(temp_EcatPeriod - (int)ECATCTRL_TASK_PERIOD);

	if(timingFlag)
	{
		if(temp_EcatPeriod > EcatPeriod)	EcatPeriod 	= temp_EcatPeriod; 
		if(temp_EcatExecution > EcatExecution)  EcatExecution 	= temp_EcatExecution;
		if(temp_EcatJitter > EcatJitter)	EcatJitter 	= temp_EcatJitter;
	}
#endif
 
#if 1 //  0 to omit : 1 for processing
	if (!(iTaskTick % FREQ_PER_SEC(ECATCTRL_TASK_PERIOD))){
		/*Do every 1 second */
		rt_printf("Task Duration: %d s Pos: %d \n", iTaskTick/FREQ_PER_SEC(ECATCTRL_TASK_PERIOD),iSlavePos);

#ifdef PERF_EVAL	   
		if(timingFlag) 
		{
			BufEcatPeriodTime[iBufEcatDataCnt] 	=   EcatPeriod;
			BufEcatExecTime[iBufEcatDataCnt]  	=   EcatExecution;
			BufEcatJitter[iBufEcatDataCnt]  	=   EcatJitter;
			++iBufEcatDataCnt;
                        EcatPeriod      = 0;
                        EcatExecution   = 0;
                        EcatJitter      = 0;


			if(iBufEcatDataCnt == BUF_SIZE)
			{
				timingFlag = 0;
				sanyoServoOp = STOP;
			}
		}
#endif //PERF_EVAL

	}
#endif


#ifdef MEASURE_TIMING	    
   	   RtmEcatPeriodEnd = RtmEcatPeriodStart;
#endif
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
		if (quitFlag) break;
	}

#ifdef PERF_EVAL
#ifdef PRINT_TIMING
	int iCnt;
	FileEcatTiming = fopen("EcatCtrl-Timing.csv", "w");
	for(iCnt=0; iCnt < iBufEcatDataCnt; ++iCnt){

		fprintf(FileEcatTiming,"%d.%06d,%d.%03d,%d.%03d\n",
				BufEcatPeriodTime[iCnt]/SCALE_1M,
				BufEcatPeriodTime[iCnt]%SCALE_1M,
				BufEcatExecTime[iCnt]/SCALE_1K,
				BufEcatExecTime[iCnt]%SCALE_1K,
				BufEcatJitter[iCnt]/SCALE_1K,
				BufEcatJitter[iCnt]%SCALE_1K);
	}
	fclose(FileEcatTiming);
#endif // PRINT_TIMING

	EcatPeriodStat = GetStatistics(BufEcatPeriodTime, iBufEcatDataCnt,SCALE_1M);  
	printf("\n[Period] Max: %.6f Min: %.6f Ave: %.6f St. D: %.6f\n", EcatPeriodStat.max,
			EcatPeriodStat.min,
			EcatPeriodStat.ave,
			EcatPeriodStat.std);
	EcatExecStat = GetStatistics(BufEcatExecTime, iBufEcatDataCnt,SCALE_1K);  
	printf("[Exec]	 Max: %.3f Min: %.3f Ave: %.3f St. D: %.3f\n", EcatExecStat.max,
			EcatExecStat.min,
			EcatExecStat.ave,
			EcatExecStat.std);
	EcatJitterStat = GetStatistics(BufEcatJitter, iBufEcatDataCnt,SCALE_1K);  
	printf("[Jitter] Max: %.3f Min: %.3f Ave: %.3f St. D: %.3f\n", EcatJitterStat.max,
			EcatJitterStat.min,
			EcatJitterStat.ave,
			EcatJitterStat.std);
#endif //PERF_EVAL

	XenoQuit();
	EcatQuit();

return ret;
}

/****************************************************************************/
void SignalHandler(int signum){
		quitFlag=1;
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
