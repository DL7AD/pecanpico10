//
//    This file is part of Dire Wolf, an amateur radio packet TNC.
//
//    Copyright (C) 2011 , 2013, 2014, 2015  John Langner, WB2OSZ
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//


/*------------------------------------------------------------------
 *
 * Name:	ax25_pad
 *
 * Purpose:	Packet assembler and disassembler.
 *
 *		This was written when I was only concerned about APRS which
 *		uses only UI frames.  ax25_pad2.c, added years later, has
 *		functions for dealing with other types of frames.
 *
 *   		We can obtain AX.25 packets from different sources:
 *		
 *		(a) from an HDLC frame.
 *		(b) from text representation.
 *		(c) built up piece by piece.
 *
 *		We also want to use a packet in different ways:
 *
 *		(a) transmit as an HDLC frame.
 *		(b) print in human-readable text.
 *		(c) take it apart piece by piece.
 *
 *		Looking at the more general case, we also want to modify
 *		an existing packet.  For instance an APRS repeater might 
 *		want to change "WIDE2-2" to "WIDE2-1" and retransmit it.
 *
 *
 * Description:	
 *
 *
 *	APRS uses only UI frames.
 *	Each starts with 2-10 addresses (14-70 octets):
 *
 *	* Destination Address  (note: opposite order in printed format)
 *
 *	* Source Address
 *
 *	* 0-8 Digipeater Addresses  (Could there ever be more as a result of
 *					digipeaters inserting their own call for
 *					the tracing feature?
 *					NO.  The limit is 8 when transmitting AX.25 over the
 *					radio.
 *					Communication with an IGate server could
 *					have a longer VIA path but that is only in text form,
 *					not as an AX.25 frame.)
 *
 *	Each address is composed of:
 *
 *	* 6 upper case letters or digits, blank padded.
 *		These are shifted left one bit, leaving the LSB always 0.
 *
 *	* a 7th octet containing the SSID and flags.
 *		The LSB is always 0 except for the last octet of the address field.
 *
 *	The final octet of the Destination has the form:
 *
 *		C R R SSID 0, where,
 *
 *			C = command/response = 1
 *			R R = Reserved = 1 1
 *			SSID = substation ID
 *			0 = zero
 *
 *		The AX.25 spec states that the RR bits should be 11 if not used.
 *		There are a couple documents talking about possible uses for APRS.
 *		I'm ignoring them for now.
 *		http://www.aprs.org/aprs12/preemptive-digipeating.txt
 *		http://www.aprs.org/aprs12/RR-bits.txt
 *
 *		I don't recall why I originally intended to set the source/destination C bits both to 1.
 *		Reviewing this 5 years later, after spending more time delving into the
 *		AX.25 spec, I think it should be 1 for destination and 0 for source.
 *		In practice you see all four combinations being used by APRS stations
 *		and no one really cares about these two bits.
 *
 *	The final octet of the Source has the form:
 *
 *		C R R SSID 0, where,
 *
 *			C = command/response = 1    (originally, now I think it should be 0 for source.)
 *						(Haven't gone back to check to see what code actually does.)
 *			R R = Reserved = 1 1
 *			SSID = substation ID
 *			0 = zero (or 1 if no repeaters)
 *
 *	The final octet of each repeater has the form:
 *
 *		H R R SSID 0, where,
 *
 *			H = has-been-repeated = 0 initially.  
 *				Set to 1 after this address has been used.
 *			R R = Reserved = 1 1
 *			SSID = substation ID
 *			0 = zero (or 1 if last repeater in list)
 *
 *		A digipeater would repeat this frame if it finds its address
 *		with the "H" bit set to 0 and all earlier repeater addresses
 *		have the "H" bit set to 1.  
 *		The "H" bit would be set to 1 in the repeated frame.
 *
 *	In standard monitoring format, an asterisk is displayed after the last
 *	digipeater with the "H" bit set.  That indicates who you are hearing
 *	over the radio.
 *	(That is if digipeaters update the via path properly.  Some don't so
 *	we don't know who we are hearing.  This is discussed in the User Guide.)
 *	No asterisk means the source is being heard directly.
 *
 *	Example, if we can hear all stations involved,
 *
 *		SRC>DST,RPT1,RPT2,RPT3:		-- we heard SRC
 *		SRC>DST,RPT1*,RPT2,RPT3:	-- we heard RPT1
 *		SRC>DST,RPT1,RPT2*,RPT3:	-- we heard RPT2
 *		SRC>DST,RPT1,RPT2,RPT3*:	-- we heard RPT3
 *
 *	
 *	Next we have:
 *
 *	* One byte Control Field 	- APRS uses 3 for UI frame
 *					   The more general AX.25 frame can have two.
 *
 *	* One byte Protocol ID 		- APRS uses 0xf0 for no layer 3
 *
 *	Finally the Information Field of 1-256 bytes.
 *
 *	And, of course, the 2 byte CRC.
 *
 * 	The descriptions above, for the C, H, and RR bits, are for APRS usage.
 *	When operating as a KISS TNC we just pass everything along and don't
 *	interpret or change them.
 *
 *
 * Constructors: ax25_init		- Clear everything.
 *		ax25_from_text		- Tear apart a text string
 *		ax25_from_frame		- Tear apart an AX.25 frame.  
 *					  Must be called before any other function.
 *
 * Get methods:	....			- Extract destination, source, or digipeater
 *					  address from frame.
 *
 * Assumptions:	CRC has already been verified to be correct.
 *
 *------------------------------------------------------------------*/

#define AX25_PAD_C		/* this will affect behavior of ax25_pad.h */

#include "ch.h"
#include "hal.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "ax25_pad.h"
#include "fcs_calc.h"
#include "debug.h"
#include "chprintf.h"
#include "pkttypes.h"


/*
 * Accumulate statistics.
 * If new_count gets much larger than delete_count plus the size of 
 * the transmit queue we have a memory leak.
 */

static volatile int new_count = 0;
static volatile int delete_count = 0;
static volatile int last_seq_num = 0;

#if AX25MEMDEBUG

int ax25memdebug = 0;


void ax25memdebug_set(void) 
{
	ax25memdebug = 1;
}

int ax25memdebug_get (void)
{
	return (ax25memdebug);
}

int ax25memdebug_seq (packet_t this_p)
{
	return (this_p->seq);
}


#endif

#define CLEAR_LAST_ADDR_FLAG  this_p->frame_data[this_p->num_addr*7-1] &= ~ SSID_LAST_MASK
#define SET_LAST_ADDR_FLAG  this_p->frame_data[this_p->num_addr*7-1] |= SSID_LAST_MASK


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_new
 * 
 * Purpose:	Allocate memory for a new packet object.
 *
 * Returns:	Identifier for a new packet object.
 *		In the current implementation this happens to be a pointer.
 *
 *------------------------------------------------------------------------------*/

packet_t ax25_new (void) {
	struct TXpacket *this_p;


#if DEBUG 
        TRACE_DEBUG("PKT  > ax25_new(): before alloc, new=%d, delete=%d", new_count, delete_count);
#endif

	last_seq_num++;
	new_count++;

/*
 * check for memory leak.
 */

// version 1.4 push up the threshold.   We could have considerably more with connected mode.

	//if (new_count > delete_count + 100) {
	if (new_count > delete_count + 256) {

#if AX25MEMDEBUG
	  // Force on debug option to gather evidence.
	  ax25memdebug_set();
#endif
	}

#if USE_CCM_HEAP_FOR_PKT == TRUE
    /* Use CCM heap. */
    extern memory_heap_t *ccm_heap;
    this_p = chHeapAlloc(ccm_heap, sizeof (struct TXpacket));
    pktAssertCCMdynamicCheck(this_p);
#else /* USE_CCM_HEAP_FOR_PKT != TRUE */
    /* Use system heap. */
    this_p = chHeapAlloc(NULL, sizeof (struct TXpacket));
#endif /* USE_CCM_HEAP_FOR_PKT == TRUE */

	if (this_p == NULL) {
	  TRACE_ERROR ("PKT  > Can't allocate memory in ax25_new.");
      return NULL;
	}

	memset(this_p, 0, sizeof(struct TXpacket));

	this_p->magic1 = MAGIC;
	this_p->seq = last_seq_num;
	this_p->magic2 = MAGIC;
	this_p->num_addr = (-1);
	this_p->nextp = NULL;

	return (this_p);
}

/*------------------------------------------------------------------------------
 *
 * Name:	ax25_delete
 * 
 * Purpose:	Destroy a packet object, freeing up memory it was using.
 *
 *------------------------------------------------------------------------------*/

#if AX25MEMDEBUG
void ax25_delete_debug (packet_t this_p, char *src_file, int src_line)
#else
void ax25_delete (packet_t this_p)
#endif
{
#if DEBUG
        TRACE_DEBUG ("PKT  > ax25_delete(): before free, new=%d, delete=%d", new_count, delete_count);
#endif

	if (this_p == NULL) {
	  TRACE_ERROR ("PKT  > ERROR - NULL pointer passed to ax25_delete.");
	  return;
	}
#if USE_CCM_HEAP_FOR_PKT == TRUE
    /* Use CCM heap. */
    pktAssertCCMdynamicCheck(this_p);
#endif

	delete_count++;

#if AX25MEMDEBUG	
	if (ax25memdebug) {
	  TRACE_DEBUG ("PKT  > ax25_delete, seq=%d, called from %s %d, new_count=%d, delete_count=%d", this_p->seq, src_file, src_line, new_count, delete_count);
	}
#endif

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
	}
	
	this_p->magic1 = 0;
	this_p->magic2 = 0;
	chHeapFree(this_p);
}


		
/*------------------------------------------------------------------------------
 *
 * Name:	ax25_from_text
 * 
 * Purpose:	Parse a frame in human-readable monitoring format and change
 *		to internal representation.
 *
 * Input:	monitor	- "TNC-2" monitor format for packet.  i.e.
 *				source>dest[,repeater1,repeater2,...]:information
 *
 *			The information part can have non-printable characters
 *			in the form of <0xff>.  This will be converted to single
 *			bytes.  e.g.  <0x0d> is carriage return.
 *			In version 1.4H we will allow nul characters which means
 *			we have to maintain a length rather than using strlen().
 *			I maintain that it violates the spec but want to handle it
 *			because it does happen and we want to preserve it when
 *			acting as an IGate rather than corrupting it.
 *
 *		strict	- True to enforce rules for packets sent over the air.
 *			  False to be more lenient for packets from IGate server.
 *
 *			  Messages from an IGate server can have longer 
 *		 	  addresses after qAC.  Up to 9 observed so far. 
 *
 *			  We can just truncate the name because we will only
 *			  end up discarding it.    TODO:  check on this.
 *
 * Returns:	Pointer to new packet object in the current implementation.
 *
 * Outputs:	Use the "get" functions to retrieve information in different ways.
 *
 *------------------------------------------------------------------------------*/

packet_t ax25_from_text (char *monitor, int strict) {

/*
 * Tearing it apart is destructive so make our own copy first.
 */
	char stuff[AX25_MAX_INFO_LEN + 1];

	char *pinfo;
	char *pa;
	char *saveptr;		/* Used with strtok_r because strtok is not thread safe. */

	int ssid_temp, heard_temp;
	char atemp[AX25_MAX_ADDR_LEN + 1];

	char info_part[AX25_MAX_INFO_LEN + 1];
	uint16_t info_len;

	packet_t this_p;
	/* Wait up to 10 seconds for a packet buffer. */
	msg_t msg = pktGetPacketBuffer(&this_p, TIME_S2I(10));
	/* If the semaphore is reset, timeout or no packet buffer then exit. */
	if(msg == MSG_RESET || msg == MSG_TIMEOUT || this_p == NULL) {
      TRACE_ERROR("PKT  > No packet buffer available");
	  return NULL;
	}

	/* Is it possible to have a nul character (zero byte) in the */
	/* information field of an AX.25 frame? */
	/* At this point, we have a normal C string. */
	/* It is possible that will convert <0x00> to a nul character later. */
	/* There we need to maintain a separate length and not use normal C string functions. */

	if(strlcpy(stuff, monitor, sizeof(stuff)) >= sizeof(stuff)) {
      TRACE_ERROR("PKT  > Source string is too large");
      pktReleaseCommonPacketBuffer(this_p);
	  return NULL;
	}

/*
 * Initialize the packet structure with two addresses and control/pid
 * for APRS.
 */
	memset (this_p->frame_data + AX25_DESTINATION * AX25_ADDR_LEN, ' ' << 1, 6);
	this_p->frame_data[AX25_DESTINATION *AX25_ADDR_LEN + 6] = SSID_H_MASK | SSID_RR_MASK;
 
	memset (this_p->frame_data + AX25_SOURCE * AX25_ADDR_LEN, ' ' << 1, 6);
	this_p->frame_data[AX25_SOURCE * AX25_ADDR_LEN + 6] = SSID_H_MASK | SSID_RR_MASK | SSID_LAST_MASK;

	this_p->frame_data[AX25_ADDR_LEN * AX25_MIN_ADDRS] = AX25_UI_FRAME;
	this_p->frame_data[AX25_ADDR_LEN * AX25_MIN_ADDRS + 1] = AX25_PID_NO_LAYER_3;

	this_p->frame_len = (AX25_ADDR_LEN * AX25_MIN_ADDRS) + 2;
	this_p->num_addr = (-1);

	if(ax25_get_num_addr(this_p) != AX25_MIN_ADDRS) {
		TRACE_ERROR("PKT  > Packet does not have required addresses after initialisation");
		pktReleaseCommonPacketBuffer(this_p);
		return NULL;
	}

/*
 * Separate the addresses from the rest.
 */
	pinfo = strchr (stuff, ':');

	if (pinfo == NULL) {
      TRACE_ERROR("PKT  > No address separator");
	  pktReleaseCommonPacketBuffer(this_p);
	  return (NULL);
	}

	*pinfo = '\0';
	pinfo++;

/*
 * Separate the addresses.
 * Note that source and destination order is swappped.
 */

/*
 * Source address.
 * Don't use traditional strtok because it is not thread safe.
 */
	pa = strtok_r (stuff, ">", &saveptr);
	if (pa == NULL) {
      TRACE_ERROR("PKT  > No source address in packet");
      /* Only need single packet release here but linked probably better for consistency. */
      pktReleaseCommonPacketBuffer(this_p);
	  return (NULL);
	}

	if ( ! ax25_parse_addr (AX25_SOURCE, pa, strict, atemp, &ssid_temp, &heard_temp)) {
      TRACE_ERROR("PKT  > Bad source address in packet");
	  /* Only need single packet release here but linked would be fine for consistency. */
      pktReleaseCommonPacketBuffer(this_p);
	  return (NULL);
	}

	ax25_set_addr (this_p, AX25_SOURCE, atemp);
	ax25_set_h (this_p, AX25_SOURCE);	// c/r in this position
	ax25_set_ssid (this_p, AX25_SOURCE, ssid_temp);

/*
 * Destination address.
 */
 
	pa = strtok_r (NULL, ",", &saveptr);
	if (pa == NULL) {
      TRACE_ERROR("PKT  > No destination address in packet");
      /* Only need single packet release here but linked probably better for consistency. */
      pktReleaseCommonPacketBuffer(this_p);
	  return (NULL);
	}

	if ( ! ax25_parse_addr (AX25_DESTINATION, pa, strict, atemp, &ssid_temp, &heard_temp)) {
      TRACE_ERROR("PKT  > Bad destination address in packet");
      /* Only need single packet release here but linked probably better for consistency. */
      pktReleaseCommonPacketBuffer(this_p);
	  return (NULL);
	}

	ax25_set_addr (this_p, AX25_DESTINATION, atemp);
	ax25_set_h (this_p, AX25_DESTINATION);	// c/r in this position
	ax25_set_ssid (this_p, AX25_DESTINATION, ssid_temp);

/*
 * VIA path.
 */
	while (( pa = strtok_r (NULL, ",", &saveptr)) != NULL && this_p->num_addr < AX25_MAX_ADDRS ) {

	  int k;

	  k = this_p->num_addr;

	  if ( ! ax25_parse_addr (k, pa, strict, atemp, &ssid_temp, &heard_temp)) {
	    TRACE_ERROR("PKT  > Bad digipeater address in packet");
	    pktReleaseCommonPacketBuffer(this_p);
	    return (NULL);
	  }

	  ax25_set_addr (this_p, k, atemp);
	  ax25_set_ssid (this_p, k, ssid_temp);

	  // Does it have an "*" at the end? 
	  // TODO: Complain if more than one "*".
	  // Could also check for all has been repeated bits are adjacent.
	
      if (heard_temp) {
	    for ( ; k >= AX25_REPEATER_1; k--) {
	      ax25_set_h (this_p, k);
	    }
	  }
    }


/*
 * Finally, process the information part.
 *
 * Translate hexadecimal values like <0xff> to single bytes.
 * MIC-E format uses 5 different non-printing characters.
 * We might want to manually generate UTF-8 characters such as degree.
 */


	info_len = 0;
	while (*pinfo != '\0' && info_len < AX25_MAX_INFO_LEN) {

	  if (strlen(pinfo) >= 6 &&
		pinfo[0] == '<' &&
		pinfo[1] == '0' &&
		pinfo[2] == 'x' &&
		isxdigit(pinfo[3]) &&
		isxdigit(pinfo[4]) &&
		pinfo[5] == '>') {

	    char *p;

	    info_part[info_len] = strtol (pinfo + 3, &p, 16);
	    info_len++;
	    pinfo += 6;
	  }
	  else {
	    info_part[info_len] = *pinfo;
	    info_len++;
	    pinfo++;
	  }
	}
	info_part[info_len] = '\0';

/*
 * Append the info part.  
 */
	/* Check for buffer overflow here. */
	if((this_p->frame_len + info_len) > AX25_MAX_PACKET_LEN) {
	  TRACE_ERROR ("PKT  > frame buffer overrun");
      pktReleaseCommonPacketBuffer(this_p);
      return (NULL);
	}
	memcpy ((char*)(this_p->frame_data + this_p->frame_len), info_part, info_len);
	this_p->frame_len += info_len;

	return (this_p);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_from_frame
 * 
 * Purpose:	Split apart an HDLC frame to components.
 *
 * Inputs:	fbuf	- Pointer to beginning of frame.
 *
 *		flen	- Length excluding the two FCS bytes.
 *
 *		alevel	- Audio level of received signal.  
 *			  Maximum range 0 - 100.
 *			  -1 might be used when not applicable.
 *
 * Returns:	Pointer to new packet object or NULL if error.
 *
 * Outputs:	Use the "get" functions to retrieve information in different ways.
 *
 *------------------------------------------------------------------------------*/

#if AX25MEMDEBUG
packet_t ax25_from_frame_debug (unsigned char *fbuf, int flen, char *src_file, int src_line)
#else
packet_t ax25_from_frame (unsigned char *fbuf, uint16_t flen)
#endif
{
	packet_t this_p;


/*
 * First make sure we have an acceptable length:
 *
 *	We are not concerned with the FCS (CRC) because someone else checked it.
 *
 * Is is possible to have zero length for info?  
 *
 * In the original version, assuming APRS, the answer was no.
 * We always had at least 3 octets after the address part:
 * control, protocol, and first byte of info part for data type.
 *
 * In later versions, this restriction was relaxed so other
 * variations of AX.25 could be used.  Now the minimum length
 * is 7+7 for addresses plus 1 for control.
 *
 */


	if (AX25_MIN_PACKET_LEN > flen || flen >= AX25_MAX_PACKET_LEN)
	{
	  TRACE_ERROR ("PKT  > Frame length %d not in allowable range of %d to %d.", flen, AX25_MIN_PACKET_LEN, AX25_MAX_PACKET_LEN);
	  return (NULL);
	}

    /* Wait up to 10 seconds for a packet buffer. */
    msg_t msg = pktGetPacketBuffer(&this_p, TIME_S2I(10));
    /* If the semaphore is reset then exit. */
    if(msg == MSG_RESET || msg == MSG_TIMEOUT || this_p == NULL) {
      TRACE_ERROR("PKT  > No packet buffer available");
      return NULL;
    }
/*	if(this_p == NULL)
	  return NULL;*/

#if AX25MEMDEBUG	
	if (ax25memdebug) {
	  TRACE_DEBUG ("PKT  > ax25_from_frame, seq=%d, called from %s %d", this_p->seq, src_file, src_line);
	}
#endif

/* Copy the whole thing intact. */

    /* Check for buffer overflow. */
    if(flen > AX25_MAX_PACKET_LEN) {
      TRACE_ERROR ("PKT  > frame buffer overrun");
      pktReleaseCommonPacketBuffer(this_p);
      return (NULL);
    }

	memcpy (this_p->frame_data, fbuf, flen);
	this_p->frame_data[flen] = 0;
	this_p->frame_len = flen;

/* Find number of addresses. */
	
	this_p->num_addr = (-1);
	(void) ax25_get_num_addr (this_p);

	return (this_p);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_dup
 * 
 * Purpose:	Make a copy of given packet object.
 *
 * Inputs:	copy_from	- Existing packet object.
 *
 * Returns:	Pointer to new packet object or NULL if error.
 *
 *
 *------------------------------------------------------------------------------*/


#if AX25MEMDEBUG
packet_t ax25_dup_debug (packet_t copy_from, char *src_file, int src_line)
#else
packet_t ax25_dup (packet_t copy_from)
#endif
{
	int save_seq;
	packet_t this_p;

	/* Wait up to 10 seconds for a packet buffer. */
	msg_t msg = pktGetPacketBuffer(&this_p, TIME_S2I(10));
    /* If the semaphore is reset then exit. */
    if(msg == MSG_RESET || msg == MSG_TIMEOUT || this_p == NULL) {
      TRACE_ERROR("PKT  > No packet buffer available");
      return NULL;
    }

	save_seq = this_p->seq;

	memcpy (this_p, copy_from, sizeof (struct TXpacket));
	this_p->seq = save_seq;

#if AX25MEMDEBUG
	if (ax25memdebug) {	
	  TRACE_DEBUG ("PKT  > ax25_dup, seq=%d, called from %s %d, clone of seq %d", this_p->seq, src_file, src_line, copy_from->seq);
	}
#endif

	return (this_p);

}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_parse_addr
 * 
 * Purpose:	Parse address with optional ssid.
 *
 * Inputs:	position	- AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER_1...
 *				  Used for more specific error message.  -1 if not used.
 *
 *		in_addr		- Input such as "WB2OSZ-15*"
 *
 * 		strict		- TRUE for strict checking (6 characters, no lower case,
 *				  SSID must be in range of 0 to 15).
 *				  Strict is appropriate for packets sent
 *				  over the radio.  Communication with IGate
 *				  allows lower case (e.g. "qAR") and two 
 *				  alphanumeric characters for the SSID.
 *				  We also get messages like this from a server.
 *					KB1POR>APU25N,TCPIP*,qAC,T2NUENGLD:...
 *
 * Outputs:	out_addr	- Address without any SSID.
 *				  Must be at least AX25_MAX_ADDR_LEN bytes.
 *
 *		out_ssid	- Numeric value of SSID.
 *
 *		out_heard	- True if "*" found.
 *
 * Returns:	True if OK, false if any error.
 *
 *
 *------------------------------------------------------------------------------*/

static const char *position_name[1 + AX25_MAX_ADDRS] = {
	"", "Destination ", "Source ",
	"Digi1 ", "Digi2 ", "Digi3 ", "Digi4 ",
	"Digi5 ", "Digi6 ", "Digi7 ", "Digi8 " };

bool ax25_parse_addr (int position, char *in_addr, int strict,
                     char *out_addr, int *out_ssid, int *out_heard) {
	char *p;
	char sstr[8];		/* Should be 1 or 2 digits for SSID. */
	int i, j, k;
	int maxlen;

	*out_addr = '\0';
	*out_ssid = 0;
	*out_heard = 0;

	if (strict && strlen(in_addr) >= 2 && strncmp(in_addr, "qA", 2) == 0) {

	  TRACE_ERROR ("PKT  > %sAddress \"%s\" is a \"q-construct\" used for communicating", position_name[position], in_addr);
	  TRACE_ERROR ("PKT  > with APRS Internet Servers.  It was not expected here.");
	}

	//printf ("ax25_parse_addr in: %s\n", in_addr);

	if (position < -1) position = -1;
	if (position > AX25_REPEATER_8) position = AX25_REPEATER_8;
	position++;	/* Adjust for position_name above. */

	maxlen = strict ? 6 : (AX25_MAX_ADDR_LEN - 1);
	p = in_addr;
	i = 0;
	for (p = in_addr; isalnum(*p); p++) {
	  if (i >= maxlen) {
	    TRACE_ERROR ("PKT  > %sAddress is too long. \"%s\" has more than %d characters.", position_name[position], in_addr, maxlen);
	    return false;
	  }
	  out_addr[i++] = *p;
	  out_addr[i] = '\0';
	  if (strict && islower(*p)) {
	    TRACE_ERROR ("PKT  > %sAddress has lower case letters. \"%s\" must be all upper case.", position_name[position], in_addr);
        return false;
	  }
	}
	
	j = 0;
	sstr[j] = '\0';
	if (*p == '-') {
	  for (p++; isalnum(*p); p++) {
	    if (j >= 2) {
	      TRACE_ERROR ("PKT  > %sSSID is too long. SSID part of \"%s\" has more than 2 characters.", position_name[position], in_addr);
	      return false;
	    }
	    sstr[j++] = *p;
	    sstr[j] = '\0';
	    if (strict && ! isdigit(*p)) {
	      TRACE_ERROR ("PKT  > %sSSID must be digits. \"%s\" has letters in SSID.", position_name[position], in_addr);
	      return false;
	    }
	  }
	  k = atoi(sstr);
	  if (k < 0 || k > 15) {
	    TRACE_ERROR ("PKT  > %sSSID out of range. SSID of \"%s\" not in range of 0 to 15.", position_name[position], in_addr);
        return false;
	  }
	  *out_ssid = k;
	}

	if (*p == '*') {
	  *out_heard = 1;
	  p++;
	}

	if (*p != '\0') {
	    TRACE_ERROR ("PKT  > Invalid character \"%c\" found in %saddress \"%s\".", *p, position_name[position], in_addr);
	      return false;
	}

	return true;

} /* end ax25_parse_addr */


/*-------------------------------------------------------------------
 *
 * Name:        ax25_check_addresses
 *
 * Purpose:     Check addresses of given packet and print message if any issues.
 *		We call this when receiving and transmitting.
 *
 * Inputs:	pp	- packet object pointer.
 *
 * Errors:	Print error message.
 *
 * Returns:	1 for all valid.  0 if not.
 *
 * Examples:	I was surprised to get this from an APRS-IS server with
 *		a lower case source address.
 *
 *			n1otx>APRS,TCPIP*,qAC,THIRD:@141335z4227.48N/07111.73W_348/005g014t044r000p000h60b10075.wview_5_20_2
 *
 *		I haven't gotten to the bottom of this yet but it sounds
 *		like "q constructs" are somehow getting on to the air when
 *		they should only appear in conversations with IGate servers.
 *
 *			https://groups.yahoo.com/neo/groups/direwolf_packet/conversations/topics/678
 *
 *			WB0VGI-7>APDW12,W0YC-5*,qAR,AE0RF-10:}N0DZQ-10>APWW10,TCPIP,WB0VGI-7*:;145.230MN*080306z4607.62N/09230.58WrKE0ACL/R 145.230- T146.2 (Pine County ARES)	
 *
 * Typical result:
 *
 *			Digipeater WIDE2 (probably N3LEE-4) audio level = 28(10/6)   [NONE]   __|||||||
 *			[0.5] VE2DJE-9>P_0_P?,VE2PCQ-3,K1DF-7,N3LEE-4,WIDE2*:'{S+l <0x1c>>/
 *			Invalid character "_" in MIC-E destination/latitude.
 *			Invalid character "_" in MIC-E destination/latitude.
 *			Invalid character "?" in MIC-E destination/latitude.
 *			Invalid MIC-E N/S encoding in 4th character of destination.
 *			Invalid MIC-E E/W encoding in 6th character of destination.
 *			MIC-E, normal car (side view), Unknown manufacturer, Returning
 *			N 00 00.0000, E 005 55.1500, 0 MPH
 *			Invalid character "_" found in Destination address "P_0_P?".
 *
 *			*** The origin and journey of this packet should receive some scrutiny. ***
 *
 *--------------------------------------------------------------------*/

int ax25_check_addresses (packet_t pp)
{
	int n;
	char addr[AX25_MAX_ADDR_LEN + 1];
	char ignore1[AX25_MAX_ADDR_LEN + 1];
	int ignore2, ignore3;
	int all_ok = 1;

	for (n = 0; n < ax25_get_num_addr(pp); n++) {
	  ax25_get_addr_with_ssid (pp, n, addr);
	  all_ok &= ax25_parse_addr (n, addr, 1, ignore1, &ignore2, &ignore3);
	}

	if (! all_ok) {
	  TRACE_ERROR ("PKT  > *** The origin and journey of this packet should receive some scrutiny. ***");
	}

	return (all_ok);
} /* end ax25_check_addresses */


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_unwrap_third_party
 * 
 * Purpose:	Unwrap a third party messge from the header.
 *
 * Inputs:	copy_from	- Existing packet object.
 *
 * Returns:	Pointer to new packet object or NULL if error.
 *
 * Example:	Input:		A>B,C:}D>E,F:info
 *		Output:		D>E,F:info
 *
 *------------------------------------------------------------------------------*/

packet_t ax25_unwrap_third_party (packet_t from_pp)
{
	unsigned char *info_p;
	packet_t result_pp;

	if (ax25_get_dti(from_pp) != '}') {
      TRACE_ERROR("PKT  > Internal error: ax25_unwrap_third_party: wrong data type.");
	  //TRACE_ERROR ("Internal error: ax25_unwrap_third_party: wrong data type.");
	  return (NULL);
	}

	if(ax25_get_info(from_pp, &info_p) == 0)
	  return NULL;

	// Want strict because addresses should conform to AX.25 here.
	// That's not the case for something from an Internet Server.

	result_pp = ax25_from_text((char *)info_p + 1, true);

	return (result_pp);
}



/*------------------------------------------------------------------------------
 *
 * Name:	ax25_set_addr
 * 
 * Purpose:	Add or change an address.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 *			  Must be either an existing address or one greater
 *			  than the final which causes a new one to be added.
 *
 *		ad	- Address with optional dash and substation id.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * TODO:  	ax25_from_text could use this.
 *
 * Returns:	None.
 *		
 *------------------------------------------------------------------------------*/

void ax25_set_addr (packet_t this_p, int n, char *ad)
{
	int ssid_temp, heard_temp;
	char atemp[AX25_MAX_ADDR_LEN + 1];
	int i;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
      TRACE_ERROR("PKT  > Buffer overflow.");
		//TRACE_ERROR("Buffer overflow");
		return;
	}
	if(n < 0 || n >= AX25_MAX_ADDRS) {
      TRACE_ERROR("PKT  > Address overflow/underflow.");
		//TRACE_ERROR("Address overflow/underflow");
		return;
	}


	if (n >= 0 && n < this_p->num_addr) {

/* 
 * Set existing address position. 
 */

	  // Why aren't we setting 'strict' here?
	  // Messages from IGate have q-constructs.
	  // We use this to parse it and later remove unwanted parts.

	  ax25_parse_addr (n, ad, 0, atemp, &ssid_temp, &heard_temp);

	  memset (this_p->frame_data + n * 7, ' ' << 1, 6);

	  for (i = 0; i < 6 && atemp[i] != '\0'; i++) {
	    this_p->frame_data[n * 7 + i] = atemp[i] << 1;
	  }
	  ax25_set_ssid (this_p, n, ssid_temp);
	}
	else if (n == this_p->num_addr) {		

/* 
 * One beyond last position, process as insert.
 */

	  ax25_insert_addr (this_p, n, ad);
	}
	else { 
      TRACE_ERROR("PKT  > Internal error, ax25_set_addr, bad position %d for '%s'", n, ad);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_insert_addr
 * 
 * Purpose:	Insert address at specified position, shifting others up one
 *		position.
 *		This is used when a digipeater wants to insert its own call
 *		for tracing purposes.
 *		For example:
 *			W1ABC>TEST,WIDE3-3
 *		Would become:
 *			W1ABC>TEST,WB2OSZ-1*,WIDE3-2
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 *		ad	- Address with optional dash and substation id.
 *
 * Bugs:	Little validity or bounds checking is performed.  Be careful.
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	None.
 *		
 *
 *------------------------------------------------------------------------------*/

void ax25_insert_addr (packet_t this_p, int n, char *ad)
{
	int ssid_temp, heard_temp;
	char atemp[AX25_MAX_ADDR_LEN + 1];
	int i;
	int expect;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}
	if(n < AX25_REPEATER_1 || n >= AX25_MAX_ADDRS) {
		TRACE_ERROR("PKT  > Address overflow");
		return;
	}

	/* Don't do it if we already have the maximum number. */
	/* Should probably return success/fail code but currently the caller doesn't care. */

	if ( this_p->num_addr >= AX25_MAX_ADDRS) {
	  return;
	}

	CLEAR_LAST_ADDR_FLAG;

	this_p->num_addr++;

	memmove (this_p->frame_data + (n+1)*7, this_p->frame_data + n*7, this_p->frame_len - (n*7));
	memset (this_p->frame_data + n*7, ' ' << 1, 6);
	this_p->frame_len += 7;
	this_p->frame_data[n*7+6] = SSID_RR_MASK;

	SET_LAST_ADDR_FLAG;

	// Why aren't we setting 'strict' here?
	// Messages from IGate have q-constructs.
	// We use this to parse it and later remove unwanted parts.

	ax25_parse_addr (n, ad, 0, atemp, &ssid_temp, &heard_temp);
	memset (this_p->frame_data + n*7, ' ' << 1, 6);
	for (i=0; i<6 && atemp[i] != '\0'; i++) {
	  this_p->frame_data[n*7+i] = atemp[i] << 1;
	}
	
	ax25_set_ssid (this_p, n, ssid_temp);

	// Sanity check after messing with number of addresses.

	expect = this_p->num_addr;
	this_p->num_addr = (-1);
	if (expect != ax25_get_num_addr (this_p)) {
	  TRACE_ERROR ("PKT  > Internal error ax25_remove_addr expect %d, actual %d", expect, this_p->num_addr);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_remove_addr
 * 
 * Purpose:	Remove address at specified position, shifting others down one position.
 *		This is used when we want to remove something from the digipeater list.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_REPEATER1, AX25_REPEATER2, etc.
 *
 * Bugs:	Little validity or bounds checking is performed.  Be careful.
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	None.
 *		
 *
 *------------------------------------------------------------------------------*/

void ax25_remove_addr (packet_t this_p, int n)
{
	int expect; 

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}
	if(n < AX25_REPEATER_1 || n >= AX25_MAX_ADDRS) {
		TRACE_ERROR("PKT  > Address overflow");
		return;
	}

	/* Shift those beyond to fill this position. */

	CLEAR_LAST_ADDR_FLAG;

	this_p->num_addr--;

	memmove (this_p->frame_data + (n * AX25_ADDR_LEN),
	         this_p->frame_data + ((n + 1) * AX25_ADDR_LEN),
	         this_p->frame_len - ((n + 1) * AX25_ADDR_LEN));
	this_p->frame_len -= AX25_ADDR_LEN;

	SET_LAST_ADDR_FLAG;

	// Sanity check after messing with number of addresses.

	expect = this_p->num_addr;
	this_p->num_addr = (-1);
	if (expect != ax25_get_num_addr (this_p)) {
	  TRACE_ERROR ("PKT  > Internal error ax25_remove_addr expect %d, actual %d", expect, this_p->num_addr);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_num_addr
 * 
 * Purpose:	Return number of addresses in current packet.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	Number of addresses in the current packet.
 *		Should be in the range of 2 .. AX25_MAX_ADDRS.
 *
 * Version 0.9:	Could be zero for a non AX.25 frame in KISS mode.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_num_addr (packet_t this_p) {
  int a;
  //int addr_bytes;


  if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
    TRACE_ERROR("PKT  > Buffer overflow");
    return 0;
  }

  /* Use cached value if already set. */

  if (this_p->num_addr >= 0) {
    return (this_p->num_addr);
  }

  /*
   * Otherwise, determine the number of addresses.
   * Start with assumption of zero.
   */

  this_p->num_addr = 0;

  /* Check that address characters are valid. */

  for(a = 0;
      a < this_p->frame_len && a < (AX25_MAX_ADDRS * AX25_ADDR_LEN);
      a++) {
    /*
     *  Check the call sign characters with isgraph
     *  Could be more strict and accept upper case alpha & numeric only.
     */
    if(a % 7 != 6) {
      if(isgraph(this_p->frame_data[a] >> 1))
        continue;
    }
    if((this_p->frame_data[a] & SSID_LAST_MASK))
      break;
  } /* End for. */

  /* Check if last happened on an address boundary. */
  if (++a % 7 == 0) {
    int addrs = a / 7;
    if (addrs >= AX25_MIN_ADDRS && addrs <= AX25_MAX_ADDRS) {
      this_p->num_addr = addrs;
    }
  }
  return (this_p->num_addr);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_num_repeaters
 * 
 * Purpose:	Return number of repeater addresses in current packet.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	Number of addresses in the current packet - 2.
 *		Should be in the range of 0 .. AX25_MAX_ADDRS - 2.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_num_repeaters (packet_t this_p)
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	if (this_p->num_addr >= 2) {
	  return (this_p->num_addr - 2);
	}

	return (0);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_addr_with_ssid
 * 
 * Purpose:	Return specified address with any SSID in current packet.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 * Outputs:	station - String representation of the station, including the SSID.
 *			e.g.  "WB2OSZ-15"
 *			  Usually variables will be AX25_MAX_ADDR_LEN bytes
 *			  but 10 would be adequate.
 *
 * Bugs:	No bounds checking is performed.  Be careful.
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	Character string in usual human readable format,
 *		
 *
 *------------------------------------------------------------------------------*/

void ax25_get_addr_with_ssid (packet_t this_p, int n, char *station)
{	
	int ssid;
	char sstr[8];		/* Should be 1 or 2 digits for SSID. */
	int i;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}


	if (n < 0) {
	  TRACE_ERROR ("PKT  > Internal error detected in ax25_get_addr_with_ssid, %s, line %d.", __FILE__, __LINE__);
	  TRACE_ERROR ("PKT  > Address index, %d, is less than zero.", n);
      strlcpy (station, "??????", AX25_MAX_SSID_ADDR_LEN + 1);
	  return;
	}

	if (n >= this_p->num_addr || n > AX25_MAX_ADDRS) {
	  TRACE_ERROR ("PKT  > Internal error detected "
	      "in ax25_get_addr_with_ssid, %s, line %d.", __FILE__, __LINE__);
	  TRACE_ERROR ("PKT  > Address index, %d, is too large or "
	      "exceeds number of addresses in this packet, %d.",
	      n, this_p->num_addr);
	  strlcpy (station, "??????", AX25_MAX_SSID_ADDR_LEN + 1);
	  return;
	}

	memset (station, 0, AX25_ADDR_LEN);
	for (i=0; i < AX25_ADDR_LEN - 1; i++) {
	  unsigned char ch;

	  ch = (this_p->frame_data[n * AX25_ADDR_LEN + i] >> 1) & 0x7f;
	  if (ch <= ' ') break;
	  station[i] = ch;
	}

	ssid = ax25_get_ssid (this_p, n);
	if (ssid != 0) {
	  chsnprintf (sstr, sizeof(sstr), "-%d", ssid);
	  strlcat (station, sstr, AX25_MAX_SSID_ADDR_LEN + 1);
	}

} /* end ax25_get_addr_with_ssid */


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_addr_no_ssid
 * 
 * Purpose:	Return specified address WITHOUT any SSID.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 * Outputs:	station - String representation of the station, WITHOUT the SSID.
 *			e.g.  "WB2OSZ"
 *			  Usually variables will be AX25_MAX_ADDR_LEN bytes
 *			  but 7 would be adequate.
 *
 * Bugs:	No bounds checking is performed.  Be careful.
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	Character string in usual human readable format,
 *		
 *
 *------------------------------------------------------------------------------*/

void ax25_get_addr_no_ssid (packet_t this_p, int n, char *station)
{	
	int i;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}


	if (n < 0) {
	  TRACE_ERROR ("PKT  > Internal error detected in ax25_get_addr_no_ssid");
	  TRACE_ERROR ("PKT  > Address index, %d, is less than zero.", n);
	  strlcpy (station, "??????", 7);
	  return;
	}

	if (n >= this_p->num_addr) {
	  TRACE_ERROR ("PKT  > Internal error detected in ax25_get_no_with_ssid");
	  TRACE_ERROR ("PKT  > Address index, %d, is too large for number of addresses, %d.", n, this_p->num_addr);
	  strlcpy (station, "??????", 7);
	  return;
	}

	memset (station, 0, AX25_ADDR_LEN);
	for (i=0; i < AX25_ADDR_LEN - 1; i++) {
	  unsigned char ch;

	  ch = (this_p->frame_data[n * AX25_ADDR_LEN + i] >> 1) & 0x7f;
	  if (ch <= ' ') break;
	  station[i] = ch;
	}

} /* end ax25_get_addr_no_ssid */


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_ssid
 * 
 * Purpose:	Return SSID of specified address in current packet.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	Substation id, as integer 0 .. 15.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_ssid (packet_t this_p, int n)
{
	
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}
	
	if (n >= 0 && n < this_p->num_addr) {
	  return ((this_p->frame_data[n * AX25_ADDR_LEN + 6] & SSID_SSID_MASK) >> SSID_SSID_SHIFT);
	}
	else {
	  TRACE_ERROR ("Internal error: ax25_get_ssid(%d), num_addr=%d", n, this_p->num_addr);
	  return (0);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_set_ssid
 * 
 * Purpose:	Set the SSID of specified address in current packet.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 *		ssid	- New SSID.  Must be in range of 0 to 15.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Bugs:	Rewrite to keep call and SSID separate internally.
 *
 *------------------------------------------------------------------------------*/

void ax25_set_ssid (packet_t this_p, int n, int ssid)
{

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}


	if (n >= 0 && n < this_p->num_addr) {
	  this_p->frame_data[n * AX25_ADDR_LEN + 6] =
	      (this_p->frame_data[n * AX25_ADDR_LEN + 6] & ~ SSID_SSID_MASK) |
		((ssid << SSID_SSID_SHIFT) & SSID_SSID_MASK) ;
	}
	else {
	  TRACE_ERROR ("PKT  > Internal error: ax25_set_ssid(%d,%d), num_addr=%d", n, ssid, this_p->num_addr);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_h
 * 
 * Purpose:	Return "has been repeated" flag of specified address in current packet.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 * Bugs:	No bounds checking is performed.  Be careful.
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	True or false.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_h (packet_t this_p, int n)
{

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}
	if(n < 0 || n >= this_p->num_addr) {
		TRACE_ERROR("PKT  > Address overflow");
		return 0;
	}

	if (n >= 0 && n < this_p->num_addr) {
	  return ((this_p->frame_data[n * AX25_ADDR_LEN + 6] & SSID_H_MASK) >> SSID_H_SHIFT);
	}
	else {
	  TRACE_ERROR ("PKT  > Internal error: ax25_get_h(%d), num_addr=%d", n, this_p->num_addr);
	  return (0);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_set_h
 * 
 * Purpose:	Set the "has been repeated" flag of specified address in current packet.
 *
 * Inputs:	n	- Index of address.   Use the symbols 
 *			 Should be in range of AX25_REPEATER_1 .. AX25_REPEATER_8.
 *
 * Bugs:	No bounds checking is performed.  Be careful.
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	None
 *
 *------------------------------------------------------------------------------*/

void ax25_set_h (packet_t this_p, int n)
{

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}

	if (n >= 0 && n < this_p->num_addr) {
	  this_p->frame_data[n * AX25_ADDR_LEN + 6] |= SSID_H_MASK;
	}
	else {
	  TRACE_ERROR ("PKT  > Internal error: ax25_set_hd(%d), num_addr=%d", n, this_p->num_addr);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_heard
 * 
 * Purpose:	Return index of the station that we heard.
 *		
 * Inputs:	none
 *
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	If any of the digipeaters have the has-been-repeated bit set, 
 *		return the index of the last one.  Otherwise return index for source.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_heard(packet_t this_p)
{
	int i;
	int result;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	result = AX25_SOURCE;

	for (i = AX25_REPEATER_1; i < ax25_get_num_addr(this_p); i++) {
	
	  if (ax25_get_h(this_p,i)) {
	    result = i;
	  }
	}
	return (result);
}



/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_first_not_repeated
 * 
 * Purpose:	Return index of the first repeater that does NOT have the 
 *		"has been repeated" flag set or -1 if none.
 *
 * Inputs:	none
 *
 *		  
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	In range of X25_REPEATER_1 .. X25_REPEATER_8 or -1 if none.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_first_not_repeated(packet_t this_p)
{
	int i;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	for (i = AX25_REPEATER_1; i < ax25_get_num_addr(this_p); i++) {
	
	  if ( ! ax25_get_h(this_p,i)) {
	    return (i);
	  }
	}
	return (-1);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_rr
 *
 * Purpose:	Return the two reserved "RR" bits in the specified address field.
 *
 * Inputs:	pp	- Packet object.
 *
 *		n	- Index of address.   Use the symbols
 *			  AX25_DESTINATION, AX25_SOURCE, AX25_REPEATER1, etc.
 *
 * Returns:	0, 1, 2, or 3.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_rr (packet_t this_p, int n)
{

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}
	if(n < 0 && n >= this_p->num_addr) {
		TRACE_ERROR("PKT  > Address overflow");
		return 0;
	}

	if (n >= 0 && n < this_p->num_addr) {
	  return ((this_p->frame_data[n*7+6] & SSID_RR_MASK) >> SSID_RR_SHIFT);
	}
	else {
	  TRACE_ERROR ("PKT  > Internal error: ax25_get_rr(%d), num_addr=%d", n, this_p->num_addr);
	  return (0);
	}
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_info
 * 
 * Purpose:	Obtain Information part of current packet.
 *
 * Inputs:	this_p	- Packet object pointer.
 *
 * Outputs:	paddr	- Starting address of information part is returned here.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	Number of octets in the Information part.
 *		Should be in the range of AX25_MIN_INFO_LEN .. AX25_MAX_INFO_LEN.
 *		Returns 0 for malformed packet or buffer overflow.
 *
 *------------------------------------------------------------------------------*/

uint16_t ax25_get_info (packet_t this_p, unsigned char **paddr) {
	unsigned char *info_ptr;
	uint16_t info_len;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	if (this_p->num_addr >= 2) {

	  /* AX.25 */

	  info_ptr = this_p->frame_data + ax25_get_info_offset(this_p);
	  info_len = ax25_get_num_info(this_p);
	}
	else {

	  /* Not AX.25.  Treat Whole packet as info. */

	  info_ptr = this_p->frame_data;
	  info_len = this_p->frame_len;
	}

    if(!info_len) {
      TRACE_WARN("PKT  > No data in packet");
	}

    /* Add null character in case caller treats as printable string. */

	info_ptr[info_len] = '\0';

	if(paddr != NULL)
	  *paddr = info_ptr;
	return (info_len);

} /* end ax25_get_info */


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_cut_at_crlf
 *
 * Purpose:	Truncate the information part at the first CR or LF.
 *		This is used for the RF>IS IGate function.
 *		CR/LF is used as record separator so we must remove it
 *		before packaging up packet to sending to server.
 *
 * Inputs:	this_p	- Packet object pointer.
 *
 * Outputs:	Packet is modified in place.
 *
 * Returns:	Number of characters removed from the end.
 *		0 if not changed.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 *------------------------------------------------------------------------------*/

int ax25_cut_at_crlf (packet_t this_p)
{
	unsigned char *info_ptr;
	int info_len;
	int j;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	if((info_len = ax25_get_info (this_p, &info_ptr)) == 0)
	    return 0;

	// Can't use strchr because there is potential of nul character.

	for (j = 0; j < info_len; j++) {

	  if (info_ptr[j] == '\r' || info_ptr[j] == '\n') {

	    int chop = info_len - j;

	    this_p->frame_len -= chop;
	    return (chop);
	  }
	}

	return (0);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_dti
 * 
 * Purpose:	Get Data Type Identifier from Information part.
 *
 * Inputs:	None.
 *
 * Assumption:	ax25_from_text or ax25_from_frame was called first.
 *
 * Returns:	First byte from the information part.
 *
 *------------------------------------------------------------------------------*/

int ax25_get_dti (packet_t this_p)
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	if (this_p->num_addr >= AX25_MIN_ADDRS) {
	  return (this_p->frame_data[ax25_get_info_offset(this_p)]);
	}
	return (' ');
}

/*------------------------------------------------------------------------------
 *
 * Name:	ax25_set_nextp
 * 
 * Purpose:	Set next packet object in queue.
 *
 * Inputs:	this_p		- Current packet object.
 *
 *		next_p		- pointer to next one
 *
 * Description:	This is used to build a linked list for a queue.
 *
 *------------------------------------------------------------------------------*/

void ax25_set_nextp (packet_t this_p, packet_t next_p)
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}
	
	this_p->nextp = next_p;
}



/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_nextp
 * 
 * Purpose:	Obtain next packet object in queue.
 *
 * Inputs:	Packet object.
 *
 * Returns:	Following object in queue or NULL.
 *
 *------------------------------------------------------------------------------*/

packet_t ax25_get_nextp (packet_t this_p)
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	return (this_p->nextp);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_set_release_time
 *
 * Purpose:	Set release time
 *
 * Inputs:	this_p		- Current packet object.
 *
 *		release_time	- Time as returned by dtime_now().
 *
 *------------------------------------------------------------------------------*/

/*void ax25_set_release_time (packet_t this_p, double release_time)
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}
	
	this_p->release_time = release_time;
}*/



/*------------------------------------------------------------------------------
 *
 * Name:	ax25_get_release_time
 *
 * Purpose:	Get release time.
 *
 *------------------------------------------------------------------------------*/

/*double ax25_get_release_time (packet_t this_p)
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	return (this_p->release_time);
}*/


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_set_modulo
 *
 * Purpose:	Set modulo value for I and S frame sequence numbers.
 *
 *------------------------------------------------------------------------------*/

/*void ax25_set_modulo (packet_t this_p, int modulo)
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}

	this_p->modulo = modulo;
}*/

/*------------------------------------------------------------------
 *
 * Function:	ax25_format_addrs
 *
 * Purpose:	Format all the addresses suitable for printing.
 *
 *		The AX.25 spec refers to this as "Source Path Header" - "TNC-2" Format
 *
 * Inputs:	Current packet.
 *		
 * Outputs:	result	- All addresses combined into a single string of the form:
 *
 *				"Source > Destination [ , repeater ... ] :"
 *
 *			An asterisk is displayed after the last digipeater 
 *			with the "H" bit set.  e.g.  If we hear RPT2, 
 *
 *			SRC>DST,RPT1,RPT2*,RPT3:
 *
 *			No asterisk means the source is being heard directly.
 *			Needs to be 101 characters to avoid overflowing.
 *			(Up to 100 characters + \0)
 *
 * Errors:	No error checking so caller needs to be careful.
 *
 *
 *------------------------------------------------------------------*/

// TODO: max len for result.  buffer overflow?

void ax25_format_addrs (packet_t this_p, char *result, int16_t size)
{
	int i;
	int heard;
	char stemp[AX25_MAX_ADDR_LEN + 1];

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}
	*result = '\0';

	/* There must be at least two addresses. */

	if (this_p->num_addr < 2) {
	  return;
	}

	/* TODO: Refactor this to use a single loop and safe write to buffer. */
	ax25_get_addr_with_ssid (this_p, AX25_SOURCE, stemp);

	if(size - (strlen(stemp) + 1) < 2)
	  return;
	size -= (strlen(stemp) + 1);
	strcat (result, stemp);
	strcat (result, ">");

	ax25_get_addr_with_ssid (this_p, AX25_DESTINATION, stemp);
    if((size - strlen(stemp)) < 2)
      return;
    size -= strlen(stemp);
	strcat (result, stemp);

	heard = ax25_get_heard(this_p);

	for (i=(int)AX25_REPEATER_1; i<this_p->num_addr; i++) {
	  ax25_get_addr_with_ssid (this_p, i, stemp);
	  if(size - (strlen(stemp) + 1) < 2)
	      return;
	    size -= (strlen(stemp) + 1);
	  strcat (result, ",");
	  strcat (result, stemp);
	  if (i == heard) {
	    if(size < 2)
	      return;
	    size--;
	    strcat (result, "*");
	  }
	}
    if(size < 2)
      return;
    size--;
	strcat (result, ":");
}


/*------------------------------------------------------------------
 *
 * Function:	ax25_format_via_path
 *
 * Purpose:	Format via path addresses suitable for printing.
 *
 * Inputs:	Current packet.
 *
 *		result_size	- Number of bytes available for result.
 *				  We can have up to 8 addresses x 9 characters
 *				  plus 7 commas, possible *, and nul = 81 minimum.
 *
 * Outputs:	result	- Digipeater field addresses combined into a single string of the form:
 *
 *				"repeater, repeater ..."
 *
 *			An asterisk is displayed after the last digipeater
 *			with the "H" bit set.  e.g.  If we hear RPT2,
 *
 *			RPT1,RPT2*,RPT3
 *
 *			No asterisk means the source is being heard directly.
 *
 *------------------------------------------------------------------*/

void ax25_format_via_path (packet_t this_p, char *result, size_t result_size)
{
	int i;
	int heard;
	char stemp[AX25_MAX_ADDR_LEN + 1];

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return;
	}
	*result = '\0';

	/* Don't get upset if no addresses.  */
	/* This will allow packets that do not comply to AX.25 format. */

	if (this_p->num_addr == 0) {
	  return;
	}

	heard = ax25_get_heard(this_p);

	for (i=(int)AX25_REPEATER_1; i<this_p->num_addr; i++) {
	  if (i > (int)AX25_REPEATER_1) {
	    strlcat (result, ",", result_size);
	  }
	  ax25_get_addr_with_ssid (this_p, i, stemp);
	  strlcat (result, stemp, result_size);
	  if (i == heard) {
	    strlcat (result, "*", result_size);
	  }
	}

} /* end ax25_format_via_path */


/*------------------------------------------------------------------
 *
 * Function:	ax25_pack
 *
 * Purpose:	Put all the pieces into format ready for transmission.
 *
 * Inputs:	this_p	- pointer to packet object.
 *		
 * Outputs:	result		- Frame buffer, AX25_MAX_PACKET_LEN bytes.
 *				Should also have two extra for FCS to be
 *				added later.
 *
 * Returns:	Number of octets in the frame buffer.  
 *		Does NOT include the extra 2 for FCS.
 *
 * Errors:	Returns -1.
 *
 *------------------------------------------------------------------*/

/*int ax25_pack (packet_t this_p, unsigned char result[AX25_MAX_PACKET_LEN])
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return -1;
	}

	if(this_p->frame_len == 0 || this_p->frame_len > AX25_MAX_PACKET_LEN) {
		TRACE_ERROR("PKT  > Packet length over-/underflow");
		return -1;
	}
     Check for buffer overflow here.
    if((this_p->frame_len + info_len) > AX25_MAX_PACKET_LEN) {
      TRACE_ERROR ("PKT  > frame buffer overrun");
      pktReleasePacketBuffer(this_p);
      return (NULL);
    }

	memcpy (result, this_p->frame_data, this_p->frame_len);

	return (this_p->frame_len);
}*/



/*------------------------------------------------------------------
 *
 * Function:	ax25_frame_type
 *
 * Purpose:	Extract the type of frame.
 *		This is derived from the control byte(s) but
 *		is an enumerated type for easier handling.
 *
 * Inputs:	this_p	- pointer to packet object.
 *		
 * Outputs:	desc	- Text description such as "I frame" or
 *			  "U frame SABME".   
 *			  Supply 40 bytes to be safe.
 *
 *		cr	- Command or response?
 *
 *		pf	- P/F - Poll/Final or -1 if not applicable
 *
 *		nr	- N(R) - receive sequence or -1 if not applicable.
 *
 *		ns	- N(S) - send sequence or -1 if not applicable.
 *	
 * Returns:	Frame type from  enum ax25_frame_type_e.
 *
 *------------------------------------------------------------------*/

// TODO: Actually unused so deprecate.
#define DESC_SIZ 40


ax25_frame_type_t ax25_frame_type (packet_t this_p, cmdres_t *cr, char *desc, int *pf, int *nr, int *ns) 
{
	int c;		// U frames are always one control byte.
	int c2 = 0;	// I & S frames can have second Control byte.

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	strlcpy (desc, "????", DESC_SIZ);
	*cr = cr_11;
	*pf = -1;
	*nr = -1;
	*ns = -1;

	c = ax25_get_control(this_p);
	if (c < 0) {
	  strlcpy (desc, "Not AX.25", DESC_SIZ);
	  return (frame_not_AX25);
	}

/*
 * TERRIBLE HACK :-(  for display purposes.
 *
 * I and S frames can have 1 or 2 control bytes but there is
 * no good way to determine this without dipping into the data
 * link state machine.  Can we guess?
 *
 * S frames have no protocol id or information so if there is one
 * more byte beyond the control field, we could assume there are
 * two control bytes.
 *
 * For I frames, the protocol id will usually be 0xf0.  If we find
 * that as the first byte of the information field, it is probably
 * the pid and not part of the information.  Ditto for segments 0x08.
 * Not fool proof but good enough for troubleshooting text out.
 *
 * If we have a link to the peer station, this will be set properly
 * before it needs to be used for other reasons.
 *
 * Setting one of the RR bits (find reference!) is sounding better and better.
 * It's in common usage so I should lobby to get that in the official protocol spec.
 */

	if (this_p->modulo == 0 && (c & 3) == 1 && ax25_get_c2(this_p) != -1) {
	  this_p->modulo = modulo_128;
	}
	else if (this_p->modulo == 0 && (c & 1) == 0 && this_p->frame_data[ax25_get_info_offset(this_p)] == 0xF0) {
	  this_p->modulo = modulo_128;
	}
	else if (this_p->modulo == 0 && (c & 1) == 0 && this_p->frame_data[ax25_get_info_offset(this_p)] == 0x08) {	// same for segments
	  this_p->modulo = modulo_128;
	}


	if (this_p->modulo == modulo_128) {
	  c2 = ax25_get_c2 (this_p);
	}

	int dst_c = this_p->frame_data[AX25_DESTINATION * AX25_ADDR_LEN + AX25_ADDR_LEN - 1] & SSID_H_MASK;
	int src_c = this_p->frame_data[AX25_SOURCE * AX25_ADDR_LEN + AX25_ADDR_LEN - 1] & SSID_H_MASK;

	char cr_text[8];
	char pf_text[8];

	if (dst_c) {
	  if (src_c) { *cr = cr_11;  strcpy(cr_text,"cc=11"); strcpy(pf_text,"p/f"); }
	  else       { *cr = cr_cmd; strcpy(cr_text,"cmd");   strcpy(pf_text,"p"); }
	}
	else {
	  if (src_c) { *cr = cr_res; strcpy(cr_text,"res");   strcpy(pf_text,"f"); }
	  else       { *cr = cr_00;  strcpy(cr_text,"cc=00"); strcpy(pf_text,"p/f"); }
	}

	if ((c & 1) == 0) {

// Information 			rrr p sss 0		or	sssssss 0  rrrrrrr p

	  if (this_p->modulo == modulo_128) {
	    *ns = (c >> 1) & 0x7f;
	    *pf = c2 & 1;
	    *nr = (c2 >> 1) & 0x7f;
	  }
	  else {
	    *ns = (c >> 1) & 7;
	    *pf = (c >> 4) & 1;
	    *nr = (c >> 5) & 7;
	  }

	  chsnprintf (desc, DESC_SIZ, "I %s, n(s)=%d, n(r)=%d, %s=%d, pid=0x%02x", cr_text, *ns, *nr, pf_text, *pf, ax25_get_pid(this_p));
	  return (frame_type_I);
	}
	else if ((c & 2) == 0) {

// Supervisory			rrr p/f ss 0 1		or	0000 ss 0 1  rrrrrrr p/f

	  if (this_p->modulo == modulo_128) {
	    *pf = c2 & 1;
	    *nr = (c2 >> 1) & 0x7f;
	  }
	  else {
	    *pf = (c >> 4) & 1;
	    *nr = (c >> 5) & 7;
	  }

 
	  switch ((c >> 2) & 3) {
	    case 0: chsnprintf (desc, DESC_SIZ, "RR %s, n(r)=%d, %s=%d", cr_text, *nr, pf_text, *pf);   return (frame_type_S_RR);   break;
	    case 1: chsnprintf (desc, DESC_SIZ, "RNR %s, n(r)=%d, %s=%d", cr_text, *nr, pf_text, *pf);  return (frame_type_S_RNR);  break;
	    case 2: chsnprintf (desc, DESC_SIZ, "REJ %s, n(r)=%d, %s=%d", cr_text, *nr, pf_text, *pf);  return (frame_type_S_REJ);  break;
	    case 3: chsnprintf (desc, DESC_SIZ, "SREJ %s, n(r)=%d, %s=%d", cr_text, *nr, pf_text, *pf); return (frame_type_S_SREJ); break;
	 } 
	}
	else {

// Unnumbered			mmm p/f mm 1 1

	  *pf = (c >> 4) & 1;
	  
	  switch (c & 0xef) {
	
	    case 0x6f: chsnprintf (desc, DESC_SIZ, "SABME %s, %s=%d",	cr_text, pf_text, *pf);  return (frame_type_U_SABME); break;
	    case 0x2f: chsnprintf (desc, DESC_SIZ, "SABM %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_SABM);  break;
	    case 0x43: chsnprintf (desc, DESC_SIZ, "DISC %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_DISC);  break;
	    case 0x0f: chsnprintf (desc, DESC_SIZ, "DM %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_DM);    break;
	    case 0x63: chsnprintf (desc, DESC_SIZ, "UA %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_UA);    break;
	    case 0x87: chsnprintf (desc, DESC_SIZ, "FRMR %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_FRMR);  break;
	    case 0x03: chsnprintf (desc, DESC_SIZ, "UI %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_UI);    break;
	    case 0xaf: chsnprintf (desc, DESC_SIZ, "XID %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_XID);   break;
	    case 0xe3: chsnprintf (desc, DESC_SIZ, "TEST %s, %s=%d", 	cr_text, pf_text, *pf);  return (frame_type_U_TEST);  break;
	    default:   chsnprintf (desc, DESC_SIZ, "U other???");        				 return (frame_type_U);       break;
	  }
	}

	// Should be unreachable but compiler doesn't realize that.
	// Here only to suppress "warning: control reaches end of non-void function"

	return (frame_not_AX25);

} /* end ax25_frame_type */


/*------------------------------------------------------------------
 *
 * Function:	ax25_is_aprs
 *
 * Purpose:	Is this packet APRS format?
 *
 * Inputs:	this_p	- pointer to packet object.
 *		
 * Returns:	True if this frame has the proper control
 *		octets for an APRS packet.
 *			control		3 for UI frame
 *			protocol id	0xf0 for no layer 3
 *
 *
 * Description:	Dire Wolf should be able to act as a KISS TNC for
 *		any type of AX.25 activity.  However, there are other
 *		places where we want to process only APRS.
 *		(e.g. digipeating and IGate.)
 *
 *------------------------------------------------------------------*/


int ax25_is_aprs (packet_t this_p) 
{
	int ctrl, pid, is_aprs;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	if (this_p->frame_len == 0) return(0);

	ctrl = ax25_get_control(this_p);
	pid = ax25_get_pid(this_p);

	is_aprs = this_p->num_addr >= 2 && ctrl == AX25_UI_FRAME && pid == AX25_PID_NO_LAYER_3;

	return (is_aprs);
}


/*------------------------------------------------------------------
 *
 * Function:	ax25_is_null_frame
 *
 * Purpose:	Is this packet structure empty?
 *
 * Inputs:	this_p	- pointer to packet object.
 *
 * Returns:	True if frame data length is 0.
 *
 * Description:	This is used when we want to wake up the
 *		transmit queue processing thread but don't
 *		want to transmit a frame.
 *
 *------------------------------------------------------------------*/


int ax25_is_null_frame (packet_t this_p)
{
	int is_null;

	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	is_null = this_p->frame_len == 0;

	return (is_null);
}


/*------------------------------------------------------------------
 *
 * Function:	ax25_get_control
 		ax25_get_c2
 *
 * Purpose:	Get Control field from packet.
 *
 * Inputs:	this_p	- pointer to packet object.
 *		
 * Returns:	APRS uses AX25_UI_FRAME.
 *		This could also be used in other situations.
 *
 *------------------------------------------------------------------*/


int ax25_get_control (packet_t this_p) 
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	if (this_p->frame_len == 0) return(-1);

	if (this_p->num_addr >= AX25_MIN_ADDRS) {
	  return (this_p->frame_data[ax25_get_control_offset(this_p)]);
	}
	return (-1);
}

int ax25_get_c2 (packet_t this_p) 
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	if (this_p->frame_len == 0) return(-1);

	if (this_p->num_addr >= AX25_MIN_ADDRS) {
	  int offset2 = ax25_get_control_offset(this_p)+1;

	  if (offset2 < this_p->frame_len) {
	    return (this_p->frame_data[offset2]);
	  }
	  else {
	    return (-1);	/* attempt to go beyond the end of frame. */
	  }
	}
	return (-1);		/* not AX.25 */
}


/*------------------------------------------------------------------
 *
 * Function:	ax25_get_pid
 *
 * Purpose:	Get protocol ID from packet.
 *
 * Inputs:	this_p	- pointer to packet object.
 *		
 * Returns:	APRS uses 0xf0 for no layer 3.
 *		This could also be used in other situations.
 *
 * AX.25:	"The Protocol Identifier (PID) field appears in information
 *		 frames (I and UI) only. It identifies which kind of
 *		 Layer 3 protocol, if any, is in use."
 *
 *------------------------------------------------------------------*/


int ax25_get_pid (packet_t this_p) 
{
	if(this_p->magic1 != MAGIC || this_p->magic2 != MAGIC) {
		TRACE_ERROR("PKT  > Buffer overflow");
		return 0;
	}

	// TODO: handle 2 control byte case.
	// TODO: sanity check: is it I or UI frame?

	if (this_p->frame_len == 0) return(-1);

	if (this_p->num_addr >= 2) {
	  return (this_p->frame_data[ax25_get_pid_offset(this_p)]);
	}
	return (-1);
}


/*------------------------------------------------------------------------------
 *
 * Name:	ax25_dedupe_crc 
 * 
 * Purpose:	Calculate a checksum for the packet source, destination, and
 *		information but NOT the digipeaters.
 *		This is used for duplicate detection in the digipeater 
 *		and IGate algorithms.
 *
 * Input:	pp	- Pointer to packet object.
 *		
 * Returns:	Value which will be the same for a duplicate but very unlikely 
 *		to match a non-duplicate packet.
 *
 * Description:	For detecting duplicates, we need to look
 *			+ source station
 *			+ destination 
 *			+ information field
 *		but NOT the changing list of digipeaters.
 *
 *		Typically, only a checksum is kept to reduce memory 
 *		requirements and amount of compution for comparisons.
 *		There is a very very small probability that two unrelated 
 *		packets will result in the same checksum, and the
 *		undesired dropping of the packet.
 *
 *		There is a 1 / 65536 chance of getting a false positive match
 *		which is good enough for this application.
 *		We could reduce that with a 32 bit CRC instead of reusing
 *		code from the AX.25 frame CRC calculation.
 *
 * Version 1.3:	We exclude any trailing CR/LF at the end of the info part
 *		so we can detect duplicates that are received only over the
 *		air and those which have gone thru an IGate where the process
 *		removes any trailing CR/LF.   Example:
 *
 *		Original via RF only:
 *		W1TG-1>APU25N,N3LEE-10*,WIDE2-1:<IGATE,MSG_CNT=30,LOC_CNT=61<0x0d>
 *
 *		When we get the same thing via APRS-IS:
 *		W1TG-1>APU25N,K1FFK,WIDE2*,qAR,WB2ZII-15:<IGATE,MSG_CNT=30,LOC_CNT=61
 *
 *		(Actually there is a trailing space.  Maybe some systems
 *		change control characters to space???)
 *		Hmmmm.  I guess we should ignore trailing space as well for 
 *		duplicate detection and suppression.
 *		
 *------------------------------------------------------------------------------*/

unsigned short ax25_dedupe_crc (packet_t pp)
{
	unsigned short crc;
	char src[AX25_MAX_ADDR_LEN + 1];
	char dest[AX25_MAX_ADDR_LEN + 1];
	unsigned char *pinfo;
	int info_len;

	ax25_get_addr_with_ssid(pp, AX25_SOURCE, src);
	ax25_get_addr_with_ssid(pp, AX25_DESTINATION, dest);
	if((info_len = ax25_get_info (pp, &pinfo)) == 0)
	  return 0;

	while (info_len >= 1 && (pinfo[info_len-1] == '\r' ||
	                         pinfo[info_len-1] == '\n' ||
	                         pinfo[info_len-1] == ' ')) {

	  info_len--;
	}

	crc = 0xffff;
	crc = crc16((unsigned char *)src, strlen(src), crc);
	crc = crc16((unsigned char *)dest, strlen(dest), crc);
	crc = crc16(pinfo, info_len, crc);

	return (crc);
}

/*------------------------------------------------------------------------------
 *
 * Name:	ax25_m_m_crc 
 * 
 * Purpose:	Calculate a checksum for the packet.
 *		This is used for the multimodem duplicate detection.
 *
 * Input:	pp	- Pointer to packet object.
 *		
 * Returns:	Value which will be the same for a duplicate but very unlikely 
 *		to match a non-duplicate packet.
 *
 * Description:	For detecting duplicates, we need to look the entire packet.
 *
 *		Typically, only a checksum is kept to reduce memory 
 *		requirements and amount of compution for comparisons.
 *		There is a very very small probability that two unrelated 
 *		packets will result in the same checksum, and the
 *		undesired dropping of the packet.
 *
 * TODO: Deprecate - not used
 *
 *------------------------------------------------------------------------------*/

/*unsigned short ax25_m_m_crc (packet_t pp)
{
	unsigned short crc;
	unsigned char fbuf[AX25_MAX_PACKET_LEN];
	int flen;

	flen = ax25_pack (pp, fbuf); 

	crc = 0xffff;
	crc = crc16(fbuf, flen, crc);

	return (crc);
}*/


/*------------------------------------------------------------------
 *
 * Function:	ax25_safe_print
 *
 * Purpose:	Print given string, changing non printable characters to 
 *		hexadecimal notation.   Note that character values
 *		<DEL>, 28, 29, 30, and 31 can appear in MIC-E message.
 *
 * Inputs:	pstr	- Pointer to string.
 *
 *		len	- Number of bytes.  If < 0 we use strlen().
 *
 *		ascii_only	- Restrict output to only ASCII.
 *				  Normally we allow UTF-8.
 *		
 *		Stops after non-zero len characters or at nul.
 *
 * Returns:	none
 *
 * Description:	Print a string in a "safe" manner.
 *		Anything that is not a printable character
 *		will be converted to a hexadecimal representation.
 *		For example, a Line Feed character will appear as <0x0a>
 *		rather than dropping down to the next line on the screen.
 *
 *		ax25_from_text can accept this format.
 *
 *
 * Example:	W1MED-1>T2QP0S,N1OHZ,N8VIM*,WIDE1-1:'cQBl <0x1c>-/]<0x0d>
 *		                                          ------   ------
 *
 * Questions:	What should we do about UTF-8?  Should that be displayed
 *		as hexadecimal for troubleshooting? Maybe an option so the
 *		packet raw data is in hexadecimal but an extracted 
 *		comment displays UTF-8?  Or a command line option for only ASCII?
 *
 * Trailing space:
 *		I recently noticed a case where a packet has space character
 *		at the end.  If the last character of the line is a space,
 *		this will be displayed in hexadecimal to make it obvious.
 *			
 *------------------------------------------------------------------*/

#define MAXSAFE 500

void ax25_safe_print (char *pstr, int len, int ascii_only)
{
	int ch;
	char safe_str[MAXSAFE*6+1];
	int safe_len;

	safe_len = 0;
	safe_str[safe_len] = '\0';


	if (len < 0) 
	  len = strlen(pstr);

	if (len > MAXSAFE)
	  len = MAXSAFE;

	while (len > 0)
	{
	  ch = *((unsigned char *)pstr);

	  if (ch == ' ' && (len == 1 || pstr[1] == '\0')) {

	      chsnprintf (safe_str + safe_len, sizeof(safe_str)-safe_len, "<0x%02x>", ch);
	      safe_len += 6;
	  }
	  else if (ch < ' ' || ch == 0x7f || ch == 0xfe || ch == 0xff ||
			(ascii_only && ch >= 0x80) ) {

	      /* Control codes and delete. */
	      /* UTF-8 does not use fe and ff except in a possible */
	      /* "Byte Order Mark" (BOM) at the beginning. */

	      chsnprintf (safe_str + safe_len, sizeof(safe_str)-safe_len, "<0x%02x>", ch);
	      safe_len += 6;
	    }
	  else {
	    /* Let everything else thru so we can handle UTF-8 */
	    /* Maybe we should have an option to display 0x80 */
	    /* and above as hexadecimal. */

	    safe_str[safe_len++] = ch;
	    safe_str[safe_len] = '\0';
	  }

	  pstr++;
	  len--;
	}

// TODO1.2: should return string rather printing to remove a race condition.

	TRACE_DEBUG ("PKT  > %s", safe_str);

} /* end ax25_safe_print */


/* end ax25_pad.c */
