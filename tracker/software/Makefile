##############################################################################
# Multi-project makefile rules
#

all:
	@$(MAKE) --version
	@echo
	@echo ============== Building for pp10a ==================================
	@$(MAKE) --no-print-directory -f ./make/pp10a.make all
	@echo ====================================================================
	@echo
	@echo ============== Building for pp10b ==================================
	@$(MAKE) --no-print-directory -f ./make/pp10b.make all
	@echo ====================================================================
	@echo

clean:
	@echo
	-@$(MAKE) --no-print-directory -f ./make/pp10a.make clean
	@echo
	-@$(MAKE) --no-print-directory -f ./make/pp10b.make clean

burna:
	@echo
	-@$(MAKE) --no-print-directory -f ./make/pp10a.make burn-pp10a
	
burnb:
	@echo
	-@$(MAKE) --no-print-directory -f ./make/pp10b.make burn-pp10b
	
##############################################################################