APP_PATH:=$(realpath $(dir $(lastword $(MAKEFILE_LIST)))/..)

all: tests

SRCDIR=$(APP_PATH)/src
SUBDIRS= utils stw

GTESTPATH=$(APP_PATH)/test/deps/googletest/googletest
GMOCKPATH=$(APP_PATH)/test/deps/googletest/googlemock
ifndef ARDUINOCLIPATH
ARDUINOCLIPATH=~/prg/arduino/cli
endif
TESTOBJSEXTRA += gtest-all.o gmock-all.o
LD=g++

FULLPATHCXXTESTSRCS := $(foreach DIR,$(SUBDIRS) tests,$(wildcard $(SRCDIR)/$(DIR)/*.cxxtest))
TESTSRCS ?= $(patsubst $(SRCDIR)/%,%,$(FULLPATHCXXTESTSRCS))
TESTOBJS = $(TESTSRCS:.cxxtest=.test.o)
TESTBINS = $(TESTSRCS:.cxxtest=.test$(EXTENTION))
TESTOUTPUTS = $(TESTSRCS:.cxxtest=.testout)
TESTMD5 = $(TESTSRCS:.cxxtest=.testmd5)
TESTD = $(TESTSRCS:.cxxtest=.dtest)

INCLUDES += -I$(GTESTPATH)/include -I$(GMOCKPATH)/include -I$(GMOCKPATH) \
            -I$(APP_PATH)/src -I$(APP_PATH)/test

ARCHOPTIMIZATION = -g -fdata-sections -ffunction-sections -fPIC

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -Wno-unknown-pragmas -MD -MP -fno-stack-protector -D_GNU_SOURCE $(INCLUDES) -DGTEST=1

CFLAGS = $(CSHAREDFLAGS) -std=gnu99 \
         $(CFLAGSENV) $(CFLAGSEXTRA) \


CXXFLAGS = $(CSHAREDFLAGS) -std=c++14 -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS $(CXXFLAGSENV) \
           $(CXXFLAGSENV) $(CXXFLAGSEXTRA) \

gtest-all.o : %.o : $(GTESTPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -Wno-uninitialized -Wno-maybe-uninitialized -I$(GTESTPATH) -MMD -MF $*.d   $< -o $@

gmock-all.o : %.o : $(GMOCKPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GMOCKPATH) -MMD -MF $*.d  $< -o $@

-include $(TESTOBJS:.test.o=.dtest)

$(TESTBINS): %.test$(EXTENTION) : %.test.o $(TESTOBJSEXTRA)
	$(LD) -o $@ $(LDFLAGS) $< $(TESTOBJSEXTRA) $(SYSLIBRARIES) 

$(TESTOBJS): %.test.o : $(SRCDIR)/%.cxxtest Makefile
	$(CXX) $(CXXFLAGS) -MMD -MF $*.dtest -MT $@ -x c++ $< -o $@

# This target takes the test binary output and compares the md5sum against the
# md5sum of the previous run. If the md5sum of the test binary didn't change,
# we do not re-run the test. We tell this to make by not updating the timestmap
# of the .md5sum even though these commands run. Make will then not run the
# dependent commands (the actual test run) and assumes that the .testout is
# also up-to-date.
%.testmd5 : %.test$(EXTENTION)
	@SM="$$(md5sum $<)" ; if [ ! -f $@ ] || [ "$$SM" != "$$(<$@)" ] ; then echo replacing md5 file. old: $$(<$@) new $$SM ; echo "$$SM" > $@ ; else echo test output up-to-date for $(TARGET):$@ ; fi

%.testout : %.testmd5
	$(EMU) $(<:.testmd5=.test$(EXTENTION)) --gtest_death_test_style=threadsafe $(TESTARGS)
	touch $@


clean-gtest:
	rm -f {gtest-all,gmock-all}.{d,o,gcno}
	rm -rf $(TESTBINS) $(TESTOBJS) $(TESTOUTPUTS) $(TESTMD5) $(TESTD)

clean veryclean: clean-gtest

run-tests: $(TESTOUTPUTS) $(TESTMD5)

build-examples: build-blink build-gpioinfra

build-blink:
	$(ARDUINOCLIPATH)/arduino-cli compile -b arduino:renesas_uno:minima $(APP_PATH)/examples/Blink

build-gpioinfra:
	$(ARDUINOCLIPATH)/arduino-cli compile -b arduino:renesas_uno:minima $(APP_PATH)/examples/GpioInfra


tests: run-tests build-examples

