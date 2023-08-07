/*------------------------------------------------------------------------------
**
**	Copyright:	AXEL s.r.l. 2010
**
**	ALPLCINFO.H:	Format target info in XML
**
**-----------------------------------------------------------------------------
**	IMPORTANT:
**
**	This module has to be checked and eventually modified by the customer
**  because of:
**
**  -   align pragmas
**	-	word/byte composition and spitting
**  -   table compiler attributes 
**
**-----------------------------------------------------------------------------*/

/*	XML tag definition					*/
#define ALPLCINFO_XMLTAG_XML			"<?xml version=\"1.0\"?>"
#define ALPLCINFO_XMLTAG_TARGET_0		"<target>"
#define ALPLCINFO_XMLTAG_TARGET_1		"</target>"
#define ALPLCINFO_XMLTAG_TARGET_ID_0	"<id>"
#define ALPLCINFO_XMLTAG_TARGET_ID_1	"</id>"
#define ALPLCINFO_XMLTAG_TARGET_PROC_0	"<proc>"
#define ALPLCINFO_XMLTAG_TARGET_PROC_1	"</proc>"
#define ALPLCINFO_XMLTAG_TARGET_COMM_0	"<comm>"
#define ALPLCINFO_XMLTAG_TARGET_COMM_1	"</comm>"
#define ALPLCINFO_XMLTAG_TARGET_ALPLC_0	"<alplc>"
#define ALPLCINFO_XMLTAG_TARGET_ALPLC_1	"</alplc>"

/*	Set target ID for XML stream		*/
#if defined( ALPLC_TARGET_ID )
#if defined( PLC_RUNTIME_CONF )
#define ALPLCINFO_TARGET_ID				"%s"
#else
#define ALPLCINFO_TARGET_ID				ALPLC_TARGET_ID
#endif
#endif

/*	Set processor for XML stream		*/
#if defined( ALPLC_P_ARM9 )
#define ALPLCINFO_TARGET_PROC			"ARM9"
#elif defined( ALPLC_P_ARM_V7_M )
#define ALPLCINFO_TARGET_PROC			"ARMv7M"
#elif defined( ALPLC_P_X86 )
#define ALPLCINFO_TARGET_PROC			"x86"
#elif defined( ALPLC_P_COLDFIRE )
#define ALPLCINFO_TARGET_PROC			"ColdFire"
#elif defined( ALPLC_P_16FX )
#define ALPLCINFO_TARGET_PROC			"16FX"
#elif defined( ALPLC_P_C166 ) && defined( ALPLC_C_C166K )
#define ALPLCINFO_TARGET_PROC			"C16xK"
#endif

/*	Set comm for XML stream				*/
#if defined( ALPLC_TARGET_COMM )
#if defined( PLC_RUNTIME_CONF )
#define ALPLCINFO_TARGET_COMM			"%s"
#else
#define ALPLCINFO_TARGET_COMM			ALPLC_TARGET_COMM
#endif
#endif

/*	Set runtime version for XML stream	*/
#if defined( ALPLC_RUNTIME_BUILD )
#define ALPLCINFO_TARGET_ALPLC			ALPLC_RUNTIME_BUILD
#endif

#if defined( ALPLCINFO_TARGET_ID )
/*	Target ID info will be added to XML */
#define ALPLCINFO_XMLSEC_TARGET_ID 		ALPLCINFO_XMLTAG_TARGET_ID_0\
											ALPLCINFO_TARGET_ID\
										ALPLCINFO_XMLTAG_TARGET_ID_1
#else
#define ALPLCINFO_XMLSEC_TARGET_ID
#endif

#if defined( ALPLCINFO_TARGET_PROC )
/*	Target proc info will be added to XML */
#define ALPLCINFO_XMLSEC_TARGET_PROC 	ALPLCINFO_XMLTAG_TARGET_PROC_0\
											ALPLCINFO_TARGET_PROC\
										ALPLCINFO_XMLTAG_TARGET_PROC_1
#else
#define ALPLCINFO_XMLSEC_TARGET_PROC
#endif

#if defined( ALPLCINFO_TARGET_COMM )
/*	Target proc info will be added to XML */
#define ALPLCINFO_XMLSEC_TARGET_COMM 	ALPLCINFO_XMLTAG_TARGET_COMM_0\
											ALPLCINFO_TARGET_COMM\
										ALPLCINFO_XMLTAG_TARGET_COMM_1
#else
#define ALPLCINFO_XMLSEC_TARGET_COMM
#endif

#if defined( ALPLCINFO_TARGET_ALPLC )
/*	PLC info will be added to XML */
#define ALPLCINFO_XMLSEC_TARGET_ALPLC 	ALPLCINFO_XMLTAG_TARGET_ALPLC_0\
											ALPLCINFO_TARGET_ALPLC\
										ALPLCINFO_XMLTAG_TARGET_ALPLC_1
#else
#define ALPLCINFO_XMLSEC_TARGET_ALPLC
#endif

/*	Compose XML info string	*/
#define ALPLCINFO_XML_INFO_STREAM	ALPLCINFO_XMLTAG_XML\
										ALPLCINFO_XMLTAG_TARGET_0\
											ALPLCINFO_XMLSEC_TARGET_ID\
											ALPLCINFO_XMLSEC_TARGET_PROC\
											ALPLCINFO_XMLSEC_TARGET_COMM\
											ALPLCINFO_XMLSEC_TARGET_ALPLC\
										ALPLCINFO_XMLTAG_TARGET_1
