# List of all the board related files.
BOARDSRC = $(CONFDIR)/board/board.c

# Required include directories
BOARDINC = $(CONFDIR)/board

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
