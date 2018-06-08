##############################################################################
# Multi-project makefile rules
#

all:
	@echo
	@echo ============== Building for pp10a ==================================
	@make --no-print-directory -f ./make/pp10a.make all
	@echo ====================================================================
	@echo
	@echo ============== Building for pp10b ==================================
	@make --no-print-directory -f ./make/pp10b.make all
	@echo ====================================================================
	@echo
clean:
	@echo
	-@make --no-print-directory -f ./make/pp10a.make clean
	@echo
	-@make --no-print-directory -f ./make/pp10b.make clean
burna:
	@echo
	-@make --no-print-directory -f ./make/pp10a.make burn-pp10a
burnb:
	@echo
	-@make --no-print-directory -f ./make/pp10b.make burn-pp10b
	
##############################################################################