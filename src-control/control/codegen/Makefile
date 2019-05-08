GEN_SRCS        = AttitudeCodegen.cpp AltitudeCodegen.cpp PositionCodegen.cpp
GEN_SRCS_FULL   = $(GEN_SRCS:%=../src-generated/%)
TEMPLATES       = AttitudeCodegenTemplate.cpp AltitudeCodegenTemplate.cpp
TEMPLATES      += PositionCodegenTemplate.cpp
TEMPLATES_FULL  = $(TEMPLATES:%=../templates/%)
MATLAB_DEP      = $(wildcard *.m)

codegen : $(GEN_SRCS_FULL)

$(GEN_SRCS_FULL) : $(MATLAB_DEP) $(TEMPLATES_FULL) Makefile
	@/bin/echo -e "\e[34mGenerating ${GEN_SRCS} ...\e[0m"
	cd .. && bash codegen/Codegen.sh

.PHONY: clean
clean:
	rm -f $(GEN_SRCS_FULL)
