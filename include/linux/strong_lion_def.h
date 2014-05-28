/*
*  define some macro and constant for Strong_Lion device
*        Lijianjun:<>
*         
*         2011-03
*/
#ifndef __STRONG_LION_DEF_H__
#define __STRONG_LION_DEF_H__

#include <linux/ioctl.h> 

//device major list ,all device magic Major must define here.

//#define AUTO_DEV_NO  

#ifdef  AUTO_DEV_NO
	#define I2C_MEM_MAJOR	0
	#define IC_MAJOR		0
	#define MCR_MAJOR		0	 
	#define PM_MAJOR		0
	#define POWR_MAJOR		0
	#define PRINTER_MAJOR	0
	#define RF_MODE_MAJOR	0
	#define URF_ID_MAJOR	0
	#define SCANNER_MAJOR	0
	#define I2C_RFID_MAJOR	0
#else
	#define I2C_MEM_MAJOR		231      
	#define IC_MAJOR			243    
	#define MCR_MAJOR			241   
	#define PM_MAJOR			240     
	#define POWER_MAJOR		239  
	#define PRINTER_MAJOR		244   
	#define RF_MODE_MAJOR		242     
	#define URF_ID_MAJOR		232	
	#define SCANNER_MAJOR		233
	#define I2C_RFID_MAJOR		234
#endif


//when the driver send signal to application
//driver use the fellow struct to deliver parameter
struct pos_signal{
   char device_name[12];
   int  event_no;
};


struct proc_misc_members{
	
	/*
 	 the level of system sleep.
 	 level 1 means all the device can go to sleep except wireless modem
 	 level 2 means all the device can go to sleep 
	*/
	char sys_sleep_level; 		/*1,2*/
	
};

extern struct proc_misc_members pms;

enum{NO_PROVIED, PROVIED};

//we own a directory at /proc to give some infomation to user

#define STRONG_LION_PROC_ROOT     	"strong_lion"
#define STRONG_LION_PROC_VERSION  "strong_lion/version"

#endif //__STRONG_LION_DEF_H__

