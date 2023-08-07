/*------------------------------------------------------------------------------
**
**	Copyright:			AXEL s.r.l. 2016
**
**	AlSSCConfCheck.h:	Configuration check for SoftScope runtime
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER
**
**------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/

/*											*/
/*	Check for proper entry point definition	*/
/*											*/

#if ! defined( ALSSCINIT )
#error "Missing runtime entry point definition of ALSSCINIT"
#endif
#if ! defined( ALSSCMANAGE )
#error "Missing runtime entry point definition of ALSSCMANAGE"
#endif
#if ! defined( ALSSCACQUIRE )
#error "Missing runtime entry point definition of ALSSCACQUIRE"
#endif
#if defined( SSC_REALTIME_ACQUIRE ) && ! defined( ALSSCRTDATASWITCH ) 
#error "Missing runtime entry point definition of ALSSCRTDATASWITCH (real-time configuration)"
#endif

/*
**	Check for proper SoftScope features
*/

#if ! defined( SSC_FEATURES )
#error "Missing SSC features definition"
#endif

#if ! defined( SSC_NUM_SAMPLES )
#error "Missing definition of number of samples supported SSC_NUM_SAMPLES"
#endif

#if SSC_NUMTRACKS > 20
#error "SSC_NUMTRACKS must be <= 20"
#endif

#if SSC_NUM_SAMPLES % SSC_NUMTRACKS
#error "SSC_NUM_SAMPLES should be multiple of SSC_NUMTRACKS"
#endif

#if ! defined( SSC_TIME_BASE )
#error "Missing sample base time SSC_TIME_BASE"
#endif

#if ! defined( SSC_R_COMMAND ) || ! defined( SSC_R_DATA )
#error "Missing definition of command and/or data registers (SSC_R_COMMAND/SSC_R_DATA)"
#endif

#if ! defined( SSC_R_W_TRIGGERSTATUS )
#error "Missing definition trigger status register SSC_R_W_TRIGGERSTATUS"
#endif

#if ! defined( SSC_R_DW_SAMPLECOUNT )
#error "Missing definition samples counter register SSC_R_DW_SAMPLECOUNT"
#endif

#if ! defined( SSC_R_DW_TRIGGERPOSITION )
#error "Missing definition of trigger position register SSC_R_DW_TRIGGERPOSITION"
#endif

#if ! defined( SSC_R_DW_ACQUIREDDATAID ) || ! defined( SSC_R_DW_ACQUISITIONID )
#error "Missing acquisition ID registers SSC_R_DW_ACQUIREDDATAID and/or SSC_R_DW_ACQUISITIONID"
#endif

#if ! defined( SSC_R_W_TRIGGERREQUIREDMODE )
#error "Missing acquisition control register SSC_R_W_TRIGGERREQUIREDMODE"
#endif

#if defined( SSC_SAMPLES_BUFFER ) && defined( SSC_REALTIME_ACQUIRE )
#error "Incompatible configuration mode: real-time mode with buffer custom definition unsupported"
#endif

