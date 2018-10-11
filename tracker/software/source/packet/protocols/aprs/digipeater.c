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
#include "crx.h"

#include "ax25_pad.h"
#include "digipeater.h"
#include "dedupe.h"
#include "fcs_calc.h"
#include "debug.h"
#include "pktradio.h"






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
 *		freq		- Frequency that we are transmitting to.
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
				  

packet_t digipeat_match (radio_freq_t from_freq, packet_t pp, char *mycall_rec,
                         char *mycall_xmit, char *alias, char *wide,
                         radio_freq_t to_freq, enum preempt_e preempt,
                         char *filter_str) {
	(void)from_freq;
	(void)filter_str;

	char source[AX25_MAX_ADDR_LEN];
	int ssid;
	int r;
	char repeater[AX25_MAX_ADDR_LEN];
	int found_len;



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
	  if(result == NULL)
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

	if (dedupe_check(pp, to_freq)) {
	  char freq[50];
	  pktDisplayFrequencyCode(to_freq, freq, sizeof(freq));
	  TRACE_INFO("PKT  > Drop redundant packet for: %s", freq);
	  return NULL;
	}

/*
 * For the alias pattern, we unconditionally digipeat it once.
 * i.e.  Just replace it with MYCALL.
 *
 * My call should be an implied member of this set.
 * In this implementation, we already caught it further up.
 */
	regex(alias, repeater, &found_len);
	if(found_len) {
	   packet_t result = ax25_dup (pp);
      if(result == NULL)
        return NULL;

	  ax25_set_addr (result, r, mycall_xmit);	
	  ax25_set_h (result, r);
	  return (result);
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

		regex(alias, repeater2, &found_len);

	    if (strcmp(repeater2, mycall_rec) == 0 ||
	      found_len != 0) {
	       packet_t result = ax25_dup (pp);
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
	      } /* End switch. */

	      return (result);
	    }
 	  }
	}

/*
 * For the wide pattern, we check the ssid and decrement it.
 */
	regex(wide, repeater, &found_len);
	if (found_len != 0) {

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


/*
 * Don't repeat it if we get here.
 */

	return (NULL);
}

