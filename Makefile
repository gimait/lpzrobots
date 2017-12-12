<<<<<<< HEAD
=======

>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005-2011
#More information: see BUILDSYSTEM.txt
SHELL=/bin/bash

include Makefile.conf

USAGE   = "Try 'make help' for more..."
USAGE2  = "lpzrobots Makefile Targets:"
<<<<<<< HEAD
USAGE3  = "Usually you do: \nmake all\n see the above description for a step by step process\n"
=======
USAGE3  = " Usually you do:\n  make prepare\n  \#follow instructions to compile ODE\n  make libs \# get a cup of tea\n  sudo make install"
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl

##!help		show this help text (default)
help:
	@cat logo.txt
	@echo $(USAGE2)
	@grep -E "^\#\#\!.*" Makefile | sed -e "s/##!/   /"
	@echo -e  $(USAGE3)

.PHONY: all
##!all		does everyhing utils, selforg, ode, ode_robots and installs them
##!		 (sudo automatically)
all:
	@if [ -w $(PREFIX) ]; then $(MAKE) all_intern; else $(MAKE) SUDO=sudo all_intern; fi

.PHONY: all_intern
all_intern:
	$(MAKE) utils
	$(SUDO) $(MAKE) install_utils
	$(MAKE) selforg
	$(SUDO) $(MAKE) install_selforg
	$(MAKE) ode
	$(SUDO) $(MAKE) install_ode
	$(MAKE) ode_robots
	$(SUDO) $(MAKE) install_ode_robots
	$(MAKE) ga_tools
	$(SUDO) $(MAKE) install_ga_tools
	@echo "**** Done, you can go and compile your simulations ****"

<<<<<<< HEAD
.PHONY: conf
##!conf		reconfigure the installation prefix and type (done automatically at first call)
conf: usage
	-mv Makefile.conf Makefile.conf.bak
# automatically creates Makefile.conf since it is included
	$(MAKE) PREFIX=$(PREFIX) TYPE=$(TYPE) Makefile.conf

##!
##!******* The following targets are called by "all" in this order ************
.PHONY: utils
##!utils	   build utilitytools and tags (do that first)
utils: usage
=======
.PHONY: prepare
##!prepare	build tools and create dependency files and check for ODE
prepare: usage 
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
	-$(MAKE) guilogger
	-$(MAKE) matrixviz
	-$(MAKE) soundman
	-$(MAKE) javacontroller
	-$(MAKE) configurator
	-$(MAKE) tags
	@if test ! -e opende/Makefile; then echo -en "You need to setup ODE from first!\nYou "; else echo -n "If you want to recompile ODE you "; fi
	@echo -e "have 2 options: use a precompiled one from our webpage or\ncompile the one in opende\nFor compiling please run:\ncd opende; sh autogen.sh\n./configure --enable-release --enable-double-precision\nmake\nsudo make install\n\nOn most SUSE linux computers it's necessary to run thereafter\nsudo ldconfig\nfor a correct linking of the libode.so!\n";
	@echo "********************************************************************************"
	@echo "Don't worry if you have seen a lot of errors above."
	@echo "This is all optional stuff which is not stricly required."
<<<<<<< HEAD


.PHONY: install_utils
##!install_utils   installs the utility tools and scripts
install_utils:
	install -d $(PREFIX)/bin $(PREFIX)/lib/soundMan $(PREFIX)/share/lpzrobots $(PREFIX)/include
	-@if [ -d matrixviz/bin/matrixviz.app ]; then \
          cp matrixviz/bin/matrixviz.app/Contents/MacOS/matrixviz $(PREFIX)/bin/ && echo "===> copied matrixviz.app to $(PREFIX)/bin/"; \
         elif [ -e matrixviz/bin/matrixviz ]; then \
	   cp matrixviz/bin/matrixviz $(PREFIX)/bin/ && echo "===> copied matrixviz to $(PREFIX)/bin/"; \
	 fi
	-cd javacontroller/src && $(MAKE) PREFIX=$(PREFIX)/ install
	-@if [ -d guilogger/bin/guilogger.app ]; then \
          cp guilogger/bin/guilogger.app/Contents/MacOS/guilogger $(PREFIX)/bin/ && echo "===> copied guilogger to $(PREFIX)/bin/"; \
         elif [ -e guilogger/src/bin/guilogger ]; then \
	   cp guilogger/src/bin/guilogger $(PREFIX)/bin/ && echo "===> copied guilogger to $(PREFIX)/bin/"; \
	 else cp guilogger/bin/guilogger $(PREFIX)/bin/ && echo "===> copied guilogger to $(PREFIX)/bin/"; \
	fi
	-@if [ -e configurator/libconfigurator.a -o -e configurator/libconfigurator.so ]; then install --mode 644 configurator/libconfigurator.* $(PREFIX)/lib/ && install --mode 755 configurator/configurator-config $(PREFIX)/bin/ && echo "===> copied libconfigurator to $(PREFIX)/lib/"; fi
	-@cp -r configurator/include/configurator $(PREFIX)/include/ && echo "===> copied configurator bins, includes and libs $(PREFIX)"
	-cp soundman/class/*.class $(PREFIX)/lib/soundMan/
	-cp soundman/bin/soundMan $(PREFIX)/bin/soundMan
	sed -i -e "s|PREFIX=.*|PREFIX=$(PREFIX)|" $(PREFIX)/bin/soundMan


.PHONY: selforg
##!selforg	   compile selforg libaries in optimized and debug version
selforg: usage
	@echo "*************** Configure selforg ***************"
	$(MAKE) MODULE=selforg confsubmodule
	@echo "*************** Compile selforg *****************"
	cd selforg && $(MAKE) depend
	cd selforg && $(MAKE)


.PHONY: install_selforg
##!install_selforg install selforg
install_selforg: usage
	cd selforg && make install

.PHONY: ode
##!ode		   compile open dynamics engine in double precession (custom version)
ode:
	cd opende; sh autogen.sh && ./configure --disable-asserts --enable-shared --enable-double-precision --prefix=$(PREFIX) --disable-demos && $(MAKE) && echo "you probably want to run \"make install_ode\" now (possibly as root)"


.PHONY: install_ode
##!install_ode	   install the customized ode library (libode_dbl)
install_ode:
	@echo "*************** Install ode -double version**********"
	cd opende && $(MAKE) install

.PHONY: ode_robots
##!ode_robots	   compile ode_robots libaries in optimized and debug version
ode_robots: usage
	@echo "*************** Configure ode_robots ************"
	$(MAKE) MODULE=ode_robots confsubmodule
	@echo "*************** Compile ode_robots **************"
	cd ode_robots && $(MAKE) depend
	cd ode_robots && $(MAKE)

.PHONY: install_ode_robots
##!install_ode_robots install ode_robots
install_ode_robots: usage
	cd ode_robots && make install

.PHONY: ga_tools
##!ga_tools	   compile ga_tools libaries in optimized and debug version
ga_tools: usage
	@echo "*************** Configure ga_tools ************"
	$(MAKE) MODULE=ga_tools confsubmodule
	@echo "*************** Compile ga_tools **************"
	cd ga_tools && $(MAKE) depend
	cd ga_tools && $(MAKE)

.PHONY: install_ga_tools
##!install_ga_tools install ga_tools
install_ga_tools: usage
	cd ga_tools && make install

##!  *** You can find example simulations in $(PREFIX)/share/lpzrobots/, but copy
##!  *** them first to your home directory to work with them.
##!
##!****** UNINSTALL and CLEAN ************
=======
	@echo "Probably you want to install the ODE and OSG now "
	@echo " and then do \"make libs\" and \"(sudo) make install\""
	@echo "Usually you can use make -j 2 on multicore machines but not for the installation target."

.PHONY: conf
##!conf		configure the installation prefix and type(again)
conf: usage
	-mv Makefile.conf Makefile.conf.bak
	$(MAKE) Makefile.conf # automatically creates Makefile.conf

.PHONY: libs
##!libs		compile libaries in optimised and debug version
libs: usage
	@echo "*************** Compile selforg (optimized) *****************"
	+cd selforg && $(MAKE) clean-all && $(MAKE) opt 	
	@echo "*************** Compile ode_robots (optimized) **************"
	+cd ode_robots && $(MAKE) clean-all && $(MAKE) opt	
	@echo "*************** Compile ga_tools (optimized) **************"
	+cd ga_tools && $(MAKE) clean-all && $(MAKE) opt	
	@echo "*************** Compile selforg (debug) *********************"
	+cd selforg && $(MAKE) clean && $(MAKE) lib
	@echo "*************** Compile ode_robots (debug) ******************"
	+cd ode_robots && $(MAKE) clean && $(MAKE) lib
	@echo "*************** Compile ga_tools (debug) **************"
	+cd ga_tools && $(MAKE) clean && $(MAKE) lib	
#	@echo "*************** strip the libs             ******************"
#	-strip selforg/libselforg_opt.a 
#	-strip --only-keep-debug selforg/libselforg.a
#	-strip ode_robots/libode_robots_opt.a
#	-strip --only-keep-debug ode_robots/libode_robots.a
#	-strip ga_tools/libga_tools_opt.a
#	-strip --only-keep-debug ga_tools/libga_tools.a

.PHONY: install
##!install	install utils and possibly libs (if installation type: user)
install: usage install_utils install_libs

.PHONY: uninstall
##!uninstall	removes all the installed files again
uninstall: uninstall_intern
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl

##!clean	  removed the object files and libs
clean: usage
	-cd guilogger && $(MAKE) clean
	-cd matrixviz && $(MAKE) clean
	-cd configurator && $(MAKE) clean
	cd opende && $(MAKE) clean
	cd ode_robots && $(MAKE) clean
	cd selforg && $(MAKE) clean
	cd ga_tools && $(MAKE) clean


##!clean-all	  like clean but also removes the libraries and clears simulations
clean-all: usage
	-cd guilogger && $(MAKE) distclean
	-cd matrixviz && $(MAKE) distclean
	-cd opende && $(MAKE) distclean
	cd ode_robots && $(MAKE) clean-all
	cd ode_robots/simulations && $(MAKE) clean
	cd selforg && $(MAKE) clean-all
	cd selforg/simulations && $(MAKE) clean
	cd selforg/examples && $(MAKE) clean
	cd ga_tools && $(MAKE) clean-all
	cd ga_tools/simulations && $(MAKE) clean
	rm -f Makefile.conf

##!distclean	  see clean-all
distclean :  clean-all

<<<<<<< HEAD
.PHONY: uninstall
##!uninstall	  removes all the installed files again (except ode)
uninstall: uninstall_intern

.PHONY: uninstall_ode
##!uninstall_ode  uninstall the customized ode library
uninstall_ode:
	@echo "*************** uninstall ode -double version********"
	cd opende && make uninstall

##!****** less common targets ***********
=======
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl

.PHONY: guilogger
##!guilogger	  compile guilogger
guilogger:
	cd guilogger && ./configure && $(MAKE)

.PHONY: matrixviz
##!matrixviz	  compile matrixviz
matrixviz:
	cd matrixviz && ./configure && $(MAKE)

.PHONY: configurator
##!configurator	  compile configurator
configurator:
	configurator/configure --prefix=$(PREFIX) --system=$$System --type=$(TYPE) --static
	$(MAKE) -C configurator
	configurator/configure --prefix=$(PREFIX) --system=$$System --type=$(TYPE)
	$(MAKE) -C configurator

.PHONY: javactrl
##!javactrl	  compile javacontroller (experimental)
javactrl:
	cd javacontroller/src && $(MAKE)

.PHONY: soundman
##!soundman	  compile soundman (experimental)
soundman:
	cd soundman/src	&& javac -d ../class/ SoundMan.java SoundManipulation.java SoundManGUI.java

<<<<<<< HEAD

.PHONY: confsubmodule
confsubmodule:
	@if [ `uname` = "Linux" ]; then \
		System="LINUX"; else System="MAC"; fi; \
	if [ -n "$(MODULE)" ]; then \
	    CMD="$(MODULE)/configure --prefix=$(PREFIX) --system=$$System --type=$(TYPE)"; \
	    echo "call: $$CMD"; \
	    if ! $$CMD; then  exit 1; fi; \
            for Folder in $(MODULE)/simulations $(MODULE)/examples; do \
	        CMD="m4 -D $$System -D $(TYPE) $$Folder/Makefile.4sim.m4"; \
	        echo "call: $$CMD"; \
	        if $$CMD > "$$Folder/Makefile.4sim"; then \
	        echo -n "genenete Makefiles in: ";\
		for F in `find "$$Folder" -mindepth 2 -name Makefile.conf`; do \
		   echo -n "$$F "; \
		   cp "$$Folder/Makefile.4sim" "$${F%.conf}"; done; \
		else \
		   echo "cannot write $$Folder/Makefile.4sim"; \
	    	fi; \
		echo "...Done"; \
	    done; \
	else \
	    echo "confsubmodule called without MODULE"; exit 1;\
	fi
=======
.PHONY: install_utils
install_utils:
	-mkdir -p $(PREFIX)bin $(PREFIX)lib/soundMan $(PREFIX)share/lpzrobots
	-cd neuronviz/src && $(MAKE) install
	-cd javacontroller/src && $(MAKE) install
	-@if [ -d guilogger/bin/guilogger.app ]; then cp guilogger/bin/guilogger.app/Contents/MacOS/guilogger $(PREFIX)bin/ && echo "copied guilogger to $(PREFIX)/bin/";  else cp guilogger/bin/guilogger $(PREFIX)bin/ && echo "copied guilogger to $(PREFIX)/bin/" || echo "Could not copy guilogger binary to $(PREFIX)bin/! Please install it by hand."; fi
	-cp soundman/class/*.class $(PREFIX)lib/soundMan/
	-cp soundman/bin/soundMan $(PREFIX)bin/soundMan
	sed -i -e "s|PREFIX=.*|PREFIX=$(PREFIX)|" $(PREFIX)bin/soundMan
	-cp ode_robots/utils/feedfile.pl $(PREFIX)bin/
	-cp ode_robots/utils/encodevideo.sh $(PREFIX)bin/
	-cp ode_robots/utils/transcode2allformats.sh $(PREFIX)bin/
	-cp ode_robots/utils/selectcolumns.pl $(PREFIX)bin/
	cp -R ode_robots/osg/data $(PREFIX)share/lpzrobots/
	-find $(PREFIX)share/lpzrobots/ -type d -name "CVS" | xargs rm -r
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl


.PHONY: install_libs
install_libs:
	@echo "*************** Install selforg *********************"
	cd selforg/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) install
#	     $(PREFIX)/share/lpzrobots/ga_tools
	@echo "*************** Install ode_robots ******************"
	cd ode_robots/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) install
ifeq ($(TYPE),user)
	@echo "*************** Install ga_tools ******************"
	cp ga_tools/libga_tools.a $(PREFIX)/lib
#	cp ga_tools/libga_tools_opt.a $(PREFIX)/lib
	cp -RL ga_tools/include/ga_tools $(PREFIX)/include/
	@echo "*************** Install example simulations ******************"
	cp -RL ga_tools/simulations $(PREFIX)/share/lpzrobots/ga_tools/
	-chmod -R ugo+r $(PREFIX)/share/lpzrobots
	@echo "*************** Finished ******************"
	@echo "Make sure that the $(PREFIX)/lib directory is in our lib search path"
	@echo " and $(PREFIX)/include is searched for includes."
	@echo "You can find example simulations in $(PREFIX)/share/lpzrobots/, but copy"
	@echo " them first to your home directory to work with them."
endif

.PHONY: uninstall_intern
uninstall_intern:
	@echo "*************** Uninstall selforg *********************"
	cd selforg/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) uninstall
	@echo "*************** Uninstall ode_robots ******************"
	cd ode_robots/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) uninstall
	@echo "*************** Uninstall ga_tools ******************"
	cd ga_tools/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) uninstall
	-rm -f $(PREFIX)/bin/guilogger
	-rm -f $(PREFIX)/lib/libconfigurator.*
	-rm -f $(PREFIX)/bin/configurator-config
	-rm -fr $(PREFIX)/include/configurator
	-rm -f $(PREFIX)/bin/matrixviz
	-cd javacontroller/src && $(MAKE) PREFIX=$(PREFIX) uninstall
	-rm -f $(PREFIX)/lib/soundMan/SoundMan*.class
	-rm -f $(PREFIX)/bin/soundMan
ifeq ($(TYPE),user)
	-rm -rf $(PREFIX)/share/lpzrobots
endif
	$(MAKE) uninstall_ode


Makefile.conf:
	@bash createMakefile.conf.sh $(PREFIX) $(TYPE)


.PHONY: tags
##!tags           create TAGS file for emacs
tags:
	@if type etags; then $(MAKE) tags_internal ; else \
	 echo "etags program not found. If you use emacs install emacs-common-bin" ; fi

.PHONY: tags_internal
tags_internal:
	rm -f TAGS
	cd selforg && $(MAKE) tags
	cd ode_robots && $(MAKE) tags
#	cd ga_tools && $(MAKE) tags

.PHONY: doc
##!doc            generate doxygen documentation in html folder
doc:
	doxygen Doxyfile

.PHONY: docintern
docintern:
	date "+PROJECT_NUMBER=%2d-%B-%Y" > builddate
	cat builddate Doxyfile.intern | nice -19 doxygen -
	find doc/html -type f | xargs chmod ug+rw
	find doc/html -type f | xargs chmod o+r


.PHONY: todo
##!todo           show the marked todos in the sourcecode
todo:
	cd ode_robots && make todo
	cd selforg && make todo

usage:
# The logo was creates with >$ figlet "LPZRobots" > logo.txt
#  plus some editing
	@cat logo.txt
	@echo $(USAGE)
