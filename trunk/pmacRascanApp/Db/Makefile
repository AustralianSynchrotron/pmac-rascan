TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# In a Diamond Support Module, install
# templates like this
#
#DB += pmacRascanGSVar.template
#DB += rascan.template
DB += rascanBase.template
DB += rascanMap.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# flatten any vdb files into a template file
$(COMMON_DIR)/%.template: %.vdb $(VDB_DEPS)
	@echo "v2t converting $< to produce $@ " 
	#flatdb --disable-macro-warnings $< $@
	python /vdb2template.py $< $@ 
	

$(COMMON_DIR)/%.db: ../%.vdb $(VDB_DEPS)
	@echo "v2t converting $< to produce $@ " 
	#flatdb --disable-macro-warnings $< $@
	python /vdb2template.py $< $@

