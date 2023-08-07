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
/*                                                                          */
/* Struttura dati:                                                          */
/* 	BLKSTOR_HEADER                                                          */
/* 		n UWORD data                                                        */
/*                                                                          */
/* 	BLKSTOR_HEADER                                                          */
/* 		n UWORD data                                                        */
/*                                                                          */
/* 	...		                                                                */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "BlockStorage.h"
#include "CommonUtility.h"
#include <string.h>

//****************************************************************************
// Funzioni locali

static SWORD search_block(void  * *,ULONG *,SWORD,UWORD  * *);
static HPVOID getblockdataaddr(BLKSTOR_HEADER  * head_addr);
static SWORD findandgetdata(void  * stor_addr,ULONG stor_size,SWORD blkcode,void  * dest_addr,SWORD dest_size,void  * * data_addr);

//****************************************************************************
// Cerca un blocco valido (se blkcode=0) oppure cerca il blocco specificato
// e ne controlla la validita'
// in uscita stor_addr e stor_size sono aggiornati al punto dove si e' fermata
// la ricerca

static SWORD search_block(void  * * stor_addr,ULONG * stor_size,SWORD blkcode,UWORD  * * blk_addr)
{
	BLKSTOR_HEADER lblk;
	UWORD  * stptr;
	SLONG  * stsize;
	UWORD crc,crcloc;
	
		// converto da void * a UWORD * perche' e' + comodo
	stptr=(UWORD  *)*stor_addr;
	stsize=(SLONG  *)stor_size;
	*blk_addr=NULL;
	
	for(;;)
	{
		if(stsize!=NULL)
			if(*stsize<=0)
				break;

		crc=1;
		if(stptr[0]==BLKSTOR_SIGN_HEAD || stptr[0]==BLKSTOR_SIGN_TAIL)
		{
			SWORD lflag=FALSE;

			memcpy(&lblk,stptr,sizeof(BLKSTOR_HEADER));

			if(stsize==NULL)
				lflag=TRUE;
			else if(stptr[0]==BLKSTOR_SIGN_HEAD)
            {
                if(lblk.uwSize<=*stsize)
				    lflag=TRUE;
            }
			else
                if(sizeof(BLKSTOR_HEADER)<=*stsize)
				    lflag=TRUE;

			if(lflag)
			{
					// calcola il CRC32 dell'header appena letto azzerando i campi
					// relativi al crc, quindi prosegue il calcolo con i dati
                crcloc=lblk.uwCrc;
                lblk.uwCrc=0;

                if(stptr[0]==BLKSTOR_SIGN_HEAD)
                {
                    crc=crc16(0  ,(const UBYTE  *)&lblk,sizeof(BLKSTOR_HEADER));
                    crc=crc16(crc,(const UBYTE  *)getblockdataaddr((BLKSTOR_HEADER  *)stptr),lblk.uwSize-sizeof(BLKSTOR_HEADER));
                }
                else
                {
                    crc=crc16(0  ,(const UBYTE  *)getblockdataaddr((BLKSTOR_HEADER  *)stptr),lblk.uwSize-sizeof(BLKSTOR_HEADER));
                    crc=crc16(crc,(const UBYTE  *)&lblk,sizeof(BLKSTOR_HEADER));
                }
				
					// il risultato dev'essere 0, senno' il crc e' fallito
				crc^=crcloc;

				if(blkcode>0)		// sto cercando un blocco specifico
				{
					if(blkcode==lblk.swCode)
						if(crc==0)
							*blk_addr=stptr;
						else
                            if(stsize==NULL)
							    return BLKSTOR_ERR_CRCERROR;
				}
				else				// sto cercando il primo blocco valido
					if(crc==0)
						*blk_addr=stptr;
			}
		}

		if(stsize!=NULL)
		{
			if(crc)					// se blocco non valido allora continua la ricerca
			{						// dalla locazione successiva
				stptr++;
				(*stsize)-=sizeof(UWORD);
			}
			else					// se blocco valido skippa tutto il blocco
			{
                if(stptr[0]==BLKSTOR_SIGN_HEAD)
                {
    				stptr=&stptr[lblk.uwSize/sizeof(UWORD)];
    				*stsize-=lblk.uwSize;
                }
                else
                {
    				stptr=&stptr[sizeof(BLKSTOR_HEADER)/sizeof(UWORD)];
    				*stsize-=sizeof(BLKSTOR_HEADER);
                }
			}
			
				// aggiorno il puntatore della funzione chiamante al punto in cui sono arrivato
			*stor_addr=(void *)stptr;
		}

		if(*blk_addr!=NULL)
			return lblk.swCode;
		
		if(stsize==NULL)
			break;
	}
	
	return BLKSTOR_ERR_NOTFOUND;
}

//****************************************************************************
// ritorna l'indirizzo a cui iniziano i dati in funzione dell'indirizzo
//dell'header (x)

static HPVOID getblockdataaddr(BLKSTOR_HEADER  * head_addr)
{
    if(head_addr->uwStartSignature==BLKSTOR_SIGN_HEAD)
        return (HPVOID)&(((UBYTE  *)(head_addr))[sizeof(BLKSTOR_HEADER)]);
    else
        return (HPVOID)((ULONG)(UBYTE  *)head_addr-(head_addr->uwSize-sizeof(BLKSTOR_HEADER)));
}

//****************************************************************************
// Cerca e ritorna il primo blocco con codice specificato; se stor_size=NULL
// allora controlla solo il primo indirizzo passato come parametro xche'
// si presume che l'indirizzo punti gia' al blocco che si vuole leggere
// se dest_addr!=NULL viene anche copiato nella zona specificata
// se data_addr!=NULL viene ritornato il puntatore ai dati del blocco
// se dest_size!=0 controlla che il blocco non sia di dimensioni superiori
//		alla locazione di destinazione
// ritorna la dimensione del blocco (se >0) oppure errore (se <0)

static SWORD findandgetdata(void  * stor_addr,ULONG stor_size,SWORD blkcode,void  * dest_addr,SWORD dest_size,void  * * data_addr)
{
	BLKSTOR_HEADER lblk;
	UWORD  * ldest;
    ULONG * stsize;
	SWORD retval;
    HPVOID ldaddr;

	if(blkcode<1 || blkcode>32767)
		return BLKSTOR_ERR_INVALIDSELBLK;
	
	if(stor_size>0)
		stsize=&stor_size;
	else
		stsize=NULL;
	
	retval=search_block(&stor_addr,stsize,blkcode,&ldest);
	if(retval<0)
		return retval;
	
	memcpy(&lblk,ldest,sizeof(BLKSTOR_HEADER));

    lblk.uwSize-=sizeof(BLKSTOR_HEADER);

	if(dest_size>0 && dest_size<lblk.uwSize)
		return BLKSTOR_ERR_INVALIDSIZE;
	
    ldaddr=getblockdataaddr((BLKSTOR_HEADER  *)ldest);

	if(dest_addr!=NULL)
		memcpy((void  *)dest_addr,ldaddr,lblk.uwSize);

    if(data_addr!=NULL)
        *data_addr=ldaddr;
	
	return (SWORD)lblk.uwSize;
}

//****************************************************************************
// Cerca e ritorna il primo blocco con codice specificato

SWORD blkstor_getdata(void  * stor_addr,ULONG stor_size,SWORD blkcode,void  * dest_addr,SWORD dest_size)
{
    return findandgetdata(stor_addr, stor_size, blkcode, dest_addr, dest_size, NULL);
}

//****************************************************************************
// Cerca e ritorna l'indirizzo dei dati del primo blocco con codice specificato

SWORD blkstor_getaddr(void  * stor_addr,ULONG stor_size,SWORD blkcode,void  * * data_addr)
{
    return findandgetdata(stor_addr, stor_size, blkcode, NULL, 0, data_addr);
}

//****************************************************************************
// Enumera i blocchi in una zona di storage; aggiorna stor_addr e stor_size
// in modo che si possa chiamare la funzione ricorsivamente; ad ogni ritorno
// fornisce il codice del blocco trovato (se >0) oppure errore (se <0) e
// l'indirizzo a cui si trova (blk_addr, per ottenerlo poi si deve chiamare la
// funzione blkstor_getdata)
// la fine e' segnalata dall'errore BLKSTOR_ERR_NOTFOUND

SWORD blkstor_enumvalid(void  * * stor_addr,ULONG * stor_size,void  * * blk_addr)
{
	UWORD  * baddr;
	SWORD retval;
	
	retval=search_block(stor_addr,stor_size,0,&baddr);
	*blk_addr=(void *)baddr;
	return retval;
}

//****************************************************************************
// Crea l'header di un blocco dati; sara' cura del chiamante andare a
// memorizzare in sequenza HEADER+dati
// Viene ottimizzato il fatto di non dover spostare i dati da una parte
// all'altra

SWORD blkstor_createheader(SWORD blkcode,volatile void  * data_addr,UWORD data_size,BLKSTOR_HEADER  * head_addr)
{
	if(blkcode<1 || blkcode>32767)
		return BLKSTOR_ERR_INVALIDSELBLK;
	
	if(data_size>32767-sizeof(BLKSTOR_HEADER))
		return BLKSTOR_ERR_INVALIDSIZE;
	
	head_addr->uwStartSignature=BLKSTOR_SIGN_HEAD;
	head_addr->uwSize=data_size+sizeof(BLKSTOR_HEADER);
    head_addr->swCode=blkcode;
    head_addr->uwCrc=0;

    head_addr->uwCrc=crc16(0,(const UBYTE  *)head_addr,sizeof(BLKSTOR_HEADER));
    head_addr->uwCrc=crc16(head_addr->uwCrc,(const UBYTE  *)data_addr,data_size);

	return blkcode;
}

//****************************************************************************
// Inizializza il TAIL di un blocco dati; sara' cura del chiamante andare a
// memorizzare in sequenza dati+TAIL
// Viene ottimizzato il fatto di non dover spostare i dati da una parte
// all'altra

SWORD blkstor_tail_begin(BLKSTOR_HEADER  * head_addr,SWORD blkcode)
{
	if(blkcode<1 || blkcode>32767)
		return BLKSTOR_ERR_INVALIDSELBLK;
	
	head_addr->uwStartSignature=BLKSTOR_SIGN_TAIL;
	head_addr->uwSize=sizeof(BLKSTOR_HEADER);
    head_addr->swCode=blkcode;
    head_addr->uwCrc=0;

	return blkcode;
}

//****************************************************************************
// Aggiunge dati al blocco creato con blkstor_tail_init in modo da poter
// creare blocchi di dati in modo incrementale

SWORD blkstor_tail_addata(BLKSTOR_HEADER  * head_addr,volatile void  * data_addr,UWORD data_size)
{
	if((ULONG)head_addr->uwSize+(ULONG)data_size>32767l)
		return BLKSTOR_ERR_INVALIDSIZE;
	
	head_addr->uwSize+=data_size;
    head_addr->uwCrc=crc16(head_addr->uwCrc,(const UBYTE  *)data_addr,data_size);

	return head_addr->swCode;
}

//****************************************************************************
// Finalizza il blocco dati

SWORD blkstor_tail_end(BLKSTOR_HEADER  * head_addr)
{
    UWORD uwCrc;

    uwCrc=head_addr->uwCrc;
    head_addr->uwCrc=0;
    head_addr->uwCrc=crc16(uwCrc,(const UBYTE  *)head_addr,sizeof(BLKSTOR_HEADER));
    
	return head_addr->swCode;
}
