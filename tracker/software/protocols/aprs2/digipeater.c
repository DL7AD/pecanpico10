//
//    This file is part of Dire Wolf, an amateur radio packet TNC.
//
//    Copyright (C) 2011, 2013, 2014, 2015  John Langner, WB2OSZ
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
 * Name:	digipeater.c
 *
 * Purpose:	Act as an APRS digital repeater.
 *		Similar cdigipeater.c is for connected mode.
 *
 *
 * Description:	Decide whether the specified packet should
 *		be digipeated and make necessary modifications.
 *
 *
 * References:	APRS Protocol Reference, document version 1.0.1
 *
 *			http://www.aprs.org/doc/APRS101.PDF
 *
 *		APRS SPEC Addendum 1.1
 *
 *			http://www.aprs.org/aprs11.html
 *
 *		APRS SPEC Addendum 1.2
 *
 *			http://www.aprs.org/aprs12.html
 *
 *		"The New n-N Paradigm"
 *
 *			http://www.aprs.org/fix14439.html
 *
 *		Preemptive Digipeating  (new in version 0.8)
 *
 *			http://www.aprs.org/aprs12/preemptive-digipeating.txt
 *		
 *------------------------------------------------------------------*/

#define DIGIPEATER_C

#include "ch.h"
#include "hal.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>	/* for isdigit, isupper */
#include "regex.h"

#include "ax25_pad.h"
#include "digipeater.h"
#include "dedupe.h"
#include "fcs_calc.h"

uint8_t data[] = ":2000000082A0AEAE6260E0AC96648A90A21B7EAC9664829AAEE2AE92888A64406303F040E6\n";






/*------------------------------------------------------------------------------
 *
 * Name:	digipeat_match
 * 
 * Purpose:	A simple digipeater for APRS.
 *
 * Input:	pp		- Pointer to a packet object.
 *	
 *		mycall_rec	- Call of my station, with optional SSID,
 *				  associated with the radio channel where the 
 *				  packet was received.
 *
 *		mycall_xmit	- Call of my station, with optional SSID,
 *				  associated with the radio channel where the 
 *				  packet is to be transmitted.  Could be the same as
 *				  mycall_rec or different.
 *
 *		alias		- Compiled pattern for my station aliases or 
 *				  "trapping" (repeating only once).
 *
 *		wide		- Compiled pattern for normal WIDEn-n digipeating.
 *
 *		to_chan		- Channel number that we are transmitting to.
 *				  This is needed to maintain a history for 
 *			 	  removing duplicates during specified time period.
 *
 *		preempt		- Option for "preemptive" digipeating.
 *
 *		filter_str	- Filter expression string or NULL.
 *		
 * Returns:	Packet object for transmission or NULL.
 *		The original packet is not modified.  (with one exception, probably obsolete)
 *		We make a copy and return that modified copy!
 *		This is very important because we could digipeat from one channel to many.
 *
 * Description:	The packet will be digipeated if the next unused digipeater
 *		field matches one of the following:
 *
 *			- mycall_rec
 *			- udigi list (only once)
 *			- wide list (usual wideN-N rules)
 *
 *------------------------------------------------------------------------------*/
				  

static packet_t digipeat_match (int from_chan, packet_t pp, char *mycall_rec, char *mycall_xmit, 
				regex_t *alias, regex_t *wide, int to_chan, enum preempt_e preempt, char *filter_str)
{
	(void)from_chan;
	(void)filter_str;

	char source[AX25_MAX_ADDR_LEN];
	int ssid;
	int r;
	char repeater[AX25_MAX_ADDR_LEN];
	int err;
	char err_msg[100];




/* 
 * Find the first repeater station which doesn't have "has been repeated" set.
 *
 * r = index of the address position in the frame.
 */
	r = ax25_get_first_not_repeated(pp);

	if (r < AX25_REPEATER_1) {
	  return (NULL);
	}

	ax25_get_addr_with_ssid(pp, r, repeater);
	ssid = ax25_get_ssid(pp, r);


/*
 * First check for explicit use of my call, including SSID.
 * Someone might explicitly specify a particular path for testing purposes.
 * This will bypass the usual checks for duplicates and my call in the source.
 *
 * In this case, we don't check the history so it would be possible
 * to have a loop (of limited size) if someone constructed the digipeater paths
 * correctly.  I would expect it only for testing purposes.
 */
	
	if (strcmp(repeater, mycall_rec) == 0) {
	  packet_t result;

	  result = ax25_dup (pp);
	  (result == NULL)
        return NULL;

	  /* If using multiple radio channels, they */
	  /* could have different calls. */
	  ax25_set_addr (result, r, mycall_xmit);	
	  ax25_set_h (result, r);
	  return (result);
	}

/*
 * Don't digipeat my own.  Fixed in 1.4 dev H.
 * Alternatively we might feed everything transmitted into
 * dedupe_remember rather than only frames out of digipeater.
 */
	ax25_get_addr_with_ssid(pp, AX25_SOURCE, source);
	if (strcmp(source, mycall_rec) == 0) {
	  return (NULL);
	}


/*
 * Next try to avoid retransmitting redundant information.
 * Duplicates are detected by comparing only:
 *	- source
 *	- destination
 *	- info part
 *	- but not the via path.  (digipeater addresses)
 * A history is kept for some amount of time, typically 30 seconds.
 * For efficiency, only a checksum, rather than the complete fields
 * might be kept but the result is the same.
 * Packets transmitted recently will not be transmitted again during
 * the specified time period.
 *
 */

	if (dedupe_check(pp, to_chan)) {
//#if DEBUG
	  /* Might be useful if people are wondering why */
	  /* some are not repeated.  Might also cause confusion. */

	  TRACE_DEBUG("Digipeater: Drop redundant packet to channel %d.\n", to_chan);
//#endif
	  return NULL;
	}

/*
 * For the alias pattern, we unconditionally digipeat it once.
 * i.e.  Just replace it with MYCALL.
 *
 * My call should be an implied member of this set.
 * In this implementation, we already caught it further up.
 */
	err = regexec(alias,repeater,0,NULL,0);
	if (err == 0) {
	  packet_t result;

	  result = ax25_dup (pp);
      if(result == NULL)
        return NULL;

	  ax25_set_addr (result, r, mycall_xmit);	
	  ax25_set_h (result, r);
	  return (result);
	}
	else if (err != REG_NOMATCH) {
	  regerror(err, alias, err_msg, sizeof(err_msg));
	  TRACE_ERROR("%s\n", err_msg);
	}

/* 
 * If preemptive digipeating is enabled, try matching my call 
 * and aliases against all remaining unused digipeaters.
 */

	if (preempt != PREEMPT_OFF) {
	  int r2;

	  for (r2 = r+1; r2 < ax25_get_num_addr(pp); r2++) {
	    char repeater2[AX25_MAX_ADDR_LEN];

	    ax25_get_addr_with_ssid(pp, r2, repeater2);

	    if (strcmp(repeater2, mycall_rec) == 0 ||
	        regexec(alias,repeater2,0,NULL,0) == 0) {
	      packet_t result;

	      result = ax25_dup (pp);
          if(result == NULL)
            return NULL;

	      ax25_set_addr (result, r2, mycall_xmit);	
	      ax25_set_h (result, r2);

	      switch (preempt) {
	        case PREEMPT_DROP:	/* remove all prior */
	          while (r2 > AX25_REPEATER_1) {
	            ax25_remove_addr (result, r2-1);
 		    r2--;
	          }
	          break;

	        case PREEMPT_MARK:
	          r2--;
	          while (r2 >= AX25_REPEATER_1 && ax25_get_h(result,r2) == 0) {
	            ax25_set_h (result, r2);
 		    r2--;
	          }
	          break;

		case PREEMPT_TRACE:	/* remove prior unused */
	        default:
	          while (r2 > AX25_REPEATER_1 && ax25_get_h(result,r2-1) == 0) {
	            ax25_remove_addr (result, r2-1);
 		    r2--;
	          }
	          break;
	      }

	      return (result);
	    }
 	  }
	}

/*
 * For the wide pattern, we check the ssid and decrement it.
 */

	err = regexec(wide,repeater,0,NULL,0);
	if (err == 0) {

/*
 * If ssid == 1, we simply replace the repeater with my call and
 *	mark it as being used.
 *
 * Otherwise, if ssid in range of 2 to 7, 
 *	Decrement y and don't mark repeater as being used.
 * 	Insert own call ahead of this one for tracing if we don't already have the 
 *	maximum number of repeaters.
 */

	  if (ssid == 1) {
	    packet_t result;

	    result = ax25_dup (pp);
        if(result == NULL)
          return NULL;

 	    ax25_set_addr (result, r, mycall_xmit);	
	    ax25_set_h (result, r);
	    return (result);
	  }

	  if (ssid >= 2 && ssid <= 7) {
	    packet_t result;

	    result = ax25_dup (pp);
        if(result == NULL)
          return NULL;

	    ax25_set_ssid(result, r, ssid-1);	// should be at least 1

	    if (ax25_get_num_repeaters(pp) < AX25_MAX_REPEATERS) {
	      ax25_insert_addr (result, r, mycall_xmit);	
	      ax25_set_h (result, r);
	    }
	    return (result);
	  }
	} 
	else if (err != REG_NOMATCH) {
	  regerror(err, wide, err_msg, sizeof(err_msg));
	  TRACE_ERROR("%s\n", err_msg);
	}


/*
 * Don't repeat it if we get here.
 */

	return (NULL);
}




/*-------------------------------------------------------------------------
 *
 * Name:	main
 * 
 * Purpose:	Standalone test case for this funtionality.
 *
 * Usage:	make -f Makefile.<platform> dtest
 *		./dtest 
 *
 *------------------------------------------------------------------------*/


static char mycall[] = "DL7AD-12";
static regex_t alias_re;
static regex_t wide_re;

static enum preempt_e preempt = PREEMPT_OFF;



static bool try_digipeat(unsigned char *frame_in, int frame_in_len, unsigned char *frame_out, int *frame_out_len)
{
	if(frame_in_len > 768) // Frame is too long and would cause a buffer overflow
		return false;

	packet_t pp, result;
	char rec[1024];
	char xmit[1024];
	unsigned char *pinfo;
	int info_len;

	pp = ax25_from_frame(frame_in, frame_in_len);
    if(result == NULL)
      return false;

	ax25_format_addrs (pp, rec);
	info_len = ax25_get_info (pp, &pinfo);
	(void)info_len;
	strlcat (rec, (char*)pinfo, sizeof(rec));

	result = digipeat_match (0, pp, mycall, mycall, &alias_re, &wide_re, 0, preempt, NULL);
	ax25_delete (pp);
	
	if (result != NULL) {

	  dedupe_remember(result, 0);
	  ax25_format_addrs(result, xmit);
	  info_len = ax25_get_info(result, &pinfo);
	  strlcat(xmit, (char*)pinfo, sizeof(xmit));
      *frame_out_len = ax25_pack(result, frame_out);
	  ax25_delete(result);

      TRACE_DEBUG("Digipeat\n");
      TRACE_DEBUG("Rec\t%s\n", rec);
      TRACE_DEBUG("Xmit\t%s\n", xmit);

      return true;

	} else {

      TRACE_DEBUG("No Digipeat\n");
      TRACE_DEBUG("Rec\t%s\n", rec);
	  strlcpy (xmit, "", sizeof(xmit));

      return false;

	}
}







uint8_t intelbuffer[1024];
uint8_t out[1024];
uint8_t last;

void clearBuffer(void) {
	for(uint32_t i=0; i<sizeof(intelbuffer); i++)
	{
		intelbuffer[i] = 0;
		out[i] = 0;
	}
}

void processIntelHex(uint8_t *buffer, uint32_t n) {
	// Parse IntelHex data
	uint8_t buffer2[n/2];
    for(uint8_t count = 0; count < n/2; count++) {
        sscanf((char*)&buffer[count*2], "%2hhx", &buffer2[count]);
    }

	// Extract data from IntelHex format
	uint8_t  i_len  = buffer2[0];
	uint16_t i_addr = (buffer2[1] << 8) | buffer2[2];
	uint8_t  i_type = buffer2[3];
	uint8_t* i_data = &buffer2[4];

	if(i_type == 1) { // Found EOF

		// Check AX.25 CRC
		unsigned short actual_fcs = (intelbuffer[last-1] << 8) | intelbuffer[last-2];
		unsigned short expected_fcs = fcs_calc(intelbuffer, last-2);

		TRACE_DEBUG("Binary[IN]: ");
		for(uint32_t i=0; i<last; i++) {
			TRACE_DEBUG("%02x ", intelbuffer[i]);
		}

		int out_len = 0;
		if(actual_fcs == expected_fcs) {
			try_digipeat(intelbuffer, last-2, out, &out_len);
		} else {
			TRACE_DEBUG("Bad CRC\n");
		}

		TRACE_DEBUG("Binary[OUT]: ");
		for(int32_t i=0; i<out_len; i++) {
			TRACE_DEBUG("%02x ", out[i]);
		}

		TRACE_DEBUG("---------------------------------------------------------\n");

		clearBuffer();
	} else if(i_type == 0) { // Found Data

		memcpy(&intelbuffer[i_addr], i_data, i_len);
		last = i_addr+i_len;

	}
}

/*int main(int argc, char *argv[])
{
	int e;
	char message[256];

	dedupe_init(TIME_S2I(4));

	e = regcomp(&alias_re, "^WIDE[4-7]-[1-7]|CITYD$", REG_EXTENDED|REG_NOSUB);
	if(e != 0) {
		regerror(e, &alias_re, message, sizeof(message));
		TRACE_DEBUG("\n%s\n\n", message);
		return 1;
	}

	e = regcomp(&wide_re, "^WIDE[1-7]-[1-7]$|^TRACE[1-7]-[1-7]$|^MA[1-7]-[1-7]$", REG_EXTENDED|REG_NOSUB);
	if(e != 0) {
		regerror(e, &wide_re, message, sizeof(message));
		TRACE_DEBUG("\n%s\n\n", message);
		return 1;
	}

	uint8_t buffer[1024];
	uint32_t n;
	uint32_t j = 0;

	clearBuffer();

	for(uint32_t i=0; i<sizeof(data); i++)
	{
		switch(j)
		{
			case 0: // Looking for start of frame
				if(data[i] == ':') { // Found start of frame
					j = 1;
					n = 0;
				}
				break;
			case 1: // capturing frame
				if(data[i] == '\n' || i+1 == n) { // Found end of frame OR end of input buffer
					buffer[n] = 0;
					processIntelHex(buffer, n);
					j = 0;
				} else {
					buffer[n++] = data[i];
				}
				break;
		}
	}

	return 0; 
}*/


