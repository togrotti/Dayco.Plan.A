/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : BlockStorage.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Block storage management                                   */
/*                                                                          */
/****************************************************************************/

#ifndef _BLOCKSTORAGE_H
#define _BLOCKSTORAGE_H

#include "CommonDefines.h"

//****************************************************************************
// Codici di creazione header

#define	BLKSTOR_SIGN_HEAD	        0xf1ca
#define	BLKSTOR_SIGN_TAIL   	    0xac1f

//****************************************************************************
// Codici di errore

	// in getdata, se specificata la dimensione del blocco e se la dimensione
	// specificata e' piu' piccola di quella del blocco allora ritorna errore
#define	BLKSTOR_ERR_INVALIDSIZE		-1

	// blocco non valido x wrong crc32
#define	BLKSTOR_ERR_CRCERROR		-2

	// il codice del blocco specificato e' invalido
#define	BLKSTOR_ERR_INVALIDSELBLK	-3

	// blocco oggetto della ricerca non trovato nell'area specificata
#define	BLKSTOR_ERR_NOTFOUND		-4

//****************************************************************************
// Struttura dati: vengono usate solo word per non avere padding e allineamenti
// vari che aumentano dimensione struttura e riducono portabilita'

typedef struct
{
	UWORD uwStartSignature;		// start signature, usato durante ricerca
								// blocchi validi (per velocizzare la ricerca)
	SWORD swCode;		    	// identificativo del blocco, 0 non ammesso
    UWORD uwSize;               // dimensione totale in byte compreso header
	UWORD uwCrc;                // CRC del blocco compreso header
} BLKSTOR_HEADER;

//****************************************************************************
// Funzioni

SWORD  blkstor_getdata(void  * stor_addr,ULONG stor_size,SWORD blkcode,void  * dest_addr,SWORD dest_size);
SWORD  blkstor_getaddr(void  * stor_addr,ULONG stor_size,SWORD blkcode,void  * * data_addr);
SWORD  blkstor_enumvalid(void  * * stor_addr,ULONG * stor_size,void  * * blk_addr);

SWORD  blkstor_createheader(SWORD blkcode,volatile void  * data_addr,UWORD data_size,BLKSTOR_HEADER  * head_addr);
SWORD  blkstor_tail_begin(BLKSTOR_HEADER  * head_addr,SWORD blkcode);
SWORD  blkstor_tail_addata(BLKSTOR_HEADER  * head_addr,volatile void  * data_addr,UWORD data_size);
SWORD  blkstor_tail_end(BLKSTOR_HEADER  * head_addr);

#endif
