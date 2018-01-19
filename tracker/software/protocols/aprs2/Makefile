CC := gcc


CFLAGS += -O2

LDFLAGS += 


# ---------------------------------------- Other utilities included ------------------------------


# Provide our own copy of strlcpy and strlcat because they are not included with Linux.
# We don't need the others in that same directory.

misc.a : strlcpy.o strlcat.o
	ar -cr $@ $^	
 
strlcpy.o : strlcpy.c
	$(CC) $(CFLAGS) -I. -c -o $@ $^

strlcat.o : strlcat.c
	$(CC) $(CFLAGS) -I. -c -o $@ $^




# ----------------------------------  Application  --------------------------------


.PHONY : dtest
dtest : digipeater.c dedupe.c ax25_pad.o fcs_calc.o misc.a
	$(CC) $(CFLAGS) -DDIGITEST -o $@ $^ $(LDFLAGS)
#	./dtest
#	rm dtest

.PHONY: clean
clean :
	rm -f dtest *.o


