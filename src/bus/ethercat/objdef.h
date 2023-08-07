/*-----------------------------------------------------------------------------------------
------
------	Description
------	
------	objdef.h
------
------	CANopen over EtherCAT object dictionary
------			 																																					------
-----------------------------------------------------------------------------------------*/

#ifndef _OBJDEF_H_
#define _OBJDEF_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "sdoserv.h"
#include "ECATCommandMgr.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Standard Types
*/

#define	DEFTYPE_NULL				0x0000
#define	DEFTYPE_BOOLEAN				0x0001
#define	DEFTYPE_INTEGER8			0x0002
#define	DEFTYPE_INTEGER16			0x0003
#define	DEFTYPE_INTEGER32			0x0004
#define	DEFTYPE_UNSIGNED8			0x0005
#define	DEFTYPE_UNSIGNED16			0x0006
#define	DEFTYPE_UNSIGNED32			0x0007
#define	DEFTYPE_REAL32				0x0008
#define	DEFTYPE_VISIBLESTRING		0x0009
#define	DEFTYPE_OCTETSTRING			0x000A
#define	DEFTYPE_REAL64				0x000B
#define	DEFTYPE_UNSIGNED24			0x0016
#define	DEFTYPE_UNSIGNED40			0x0018
#define	DEFTYPE_UNSIGNED48			0x0019
#define	DEFTYPE_UNSIGNED56			0x001A
#define	DEFTYPE_UNSIGNED64			0x001B
#define	DEFTYPE_PDOMAPPING			0x0021
#define	DEFTYPE_IDENTITY			0x0023
#define	DEFTYPE_COMMAND				0x0025
#define	DEFTYPE_PDOCOMPAR			0x0027
#define	DEFTYPE_ENUM				0x0028
#define	DEFTYPE_SMPAR				0x0029
#define	DEFTYPE_RECORD				0x002A
#define	DEFTYPE_SMDIAG				0x0030
#define	DEFTYPE_BIT1				0x0030
#define	DEFTYPE_BIT2				0x0031
#define	DEFTYPE_BIT3				0x0032
#define	DEFTYPE_BIT4				0x0033
#define	DEFTYPE_BIT5				0x0034
#define	DEFTYPE_BIT6				0x0035
#define	DEFTYPE_BIT7				0x0036
#define	DEFTYPE_BIT8				0x0037

/* ECATCHANGE_START(V4.00) OBJ 1 */
#define	BOOLEAN(x)					unsigned x:1
#define	BIT1(x)						unsigned x:1
#define	BIT2(x)						unsigned x:2
#define	BIT3(x)						unsigned x:3
#define	BIT4(x)						unsigned x:4
#define	BIT5(x)						unsigned x:5
#define	BIT6(x)						unsigned x:6
#define	BIT7(x)						unsigned x:7
#define	BIT8(x)						unsigned x:8
#define	ALIGN0(x)						
#define	ALIGN1(x)					unsigned x:1;
#define	ALIGN2(x)					unsigned x:2;
#define	ALIGN3(x)					unsigned x:3;
#define	ALIGN4(x)					unsigned x:4;
#define	ALIGN5(x)					unsigned x:5;
#define	ALIGN6(x)					unsigned x:6;
#define	ALIGN7(x)					unsigned x:7;
#define	ALIGN8(x)					unsigned x:8;
#define	ALIGN9(x)					unsigned x1:1; unsigned x:8; 
#define	ALIGN10(x)					unsigned x1:2; unsigned x:8; 
#define	ALIGN11(x)					unsigned x1:3; unsigned x:8; 
#define	ALIGN12(x)					unsigned x1:4; unsigned x:8; 
#define	ALIGN13(x)					unsigned x1:5; unsigned x:8; 
#define	ALIGN14(x)					unsigned x1:6; unsigned x:8; 
#define	ALIGN15(x)					unsigned x1:7; unsigned x:8; 
/* ECATCHANGE_END(V4.00) OBJ 1 */

#define	BYTELEN_UNKNOWN				0x0000
#define	BYTELEN_BOOLEAN				0x0001
#define	BYTELEN_INTEGER8			0x0001
#define	BYTELEN_INTEGER16			0x0002
#define	BYTELEN_INTEGER32			0x0004
#define	BYTELEN_UNSIGNED8			0x0001
#define	BYTELEN_UNSIGNED16			0x0002
#define	BYTELEN_UNSIGNED32			0x0004
#define	BYTELEN_REAL32				0x0004
#define	BYTELEN_VISIBLESTRING		0x0001
#define	BYTELEN_OCTETSTRING			0x0001
#define	BYTELEN_REAL64				0x0008
#define	BYTELEN_UNSIGNED24			0x0003
#define	BYTELEN_UNSIGNED40			0x0005
#define	BYTELEN_UNSIGNED48			0x0006
#define	BYTELEN_UNSIGNED56			0x0007
#define	BYTELEN_UNSIGNED64			0x0008

/* ECATCHANGE_START(V4.00) OBJ 1 */
#define	BITLEN_BOOLEAN				0x0001
#define	BITLEN_INTEGER8				0x0008
#define	BITLEN_INTEGER16			0x0010
#define	BITLEN_INTEGER32			0x0020
#define	BITLEN_UNSIGNED8			0x0008
#define	BITLEN_UNSIGNED16			0x0010
#define	BITLEN_UNSIGNED32			0x0020
#define	BITLEN_REAL32				0x0020
#define	BITLEN_REAL64				0x0040
#define	BITLEN_UNSIGNED24			0x0018
#define	BITLEN_UNSIGNED40			0x0028
#define	BITLEN_UNSIGNED48			0x0030
#define	BITLEN_UNSIGNED56			0x0038
#define	BITLEN_UNSIGNED64			0x0040
/* ECATCHANGE_END(V4.00) OBJ 1 */

#define	SYNCTYPE_FREERUN			0x0000
#define	SYNCTYPE_SYNCHRON			0x0001
#define	SYNCTYPE_DCSYNC0			0x0002
#define	SYNCTYPE_DCSYNC1			0x0003
#define	SYNCTYPE_SM2INT				0x0022
#define	SYNCTYPE_SM3INT				0x0023
/* ECATCHANGE_START(V4.00) ECAT 1 */
#define	SYNCTYPE_FREERUNSUPP		0x0001
#define	SYNCTYPE_SYNCHRONSUPP		0x0002
#define	SYNCTYPE_DCSYNC0SUPP		0x0004
#define	SYNCTYPE_DCSYNC1SUPP		0x0008
#define	SYNCTYPE_SUBAPPLSYNC0		0x0010
#define	SYNCTYPE_TIMESVARIABLE		0x4000
#define	SYNCTYPE_FASTMODE			0x8000

#define	SMPAR_COMPLETESIZE			26

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// type definitions of objects 
//
*/

#define TSYNCMANPAR					ECATCM_TSYNCMANPAR
#define TCYCLEDIAG					ECATCM_TCYCLEDIAG

#define sSyncManOutPar				tEcatCMSMOutParam
#define sSyncManInPar				tEcatCMSMInParam
#define sCycleDiag					tEcatCMCycleDiag

#endif //_OBJDEF_H_
