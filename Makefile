#---------------------------------------------------------------------------------
# DiRT2 clone -- Nintendo 3DS build (devkitPro / libctru / citro3d + citro2d)
#
# Shape copied from the sibling projects' Makefiles (3ds-project-folder/mc and
# 3ds-project-folder/model-making), which are themselves based on the standard
# devkitPro 3ds GPU example Makefile. Deliberately NOT copying either sibling's
# project-specific machinery (blocksmith's proto/world drift guards, either
# project's network PSK handling) -- this is a from-scratch project with no
# server dependency and no reason to carry contracts that don't apply to it.
#---------------------------------------------------------------------------------
.SUFFIXES:
#---------------------------------------------------------------------------------

ifeq ($(strip $(DEVKITARM)),)
$(error "Please set DEVKITARM in your environment. export DEVKITARM=<path to>devkitARM")
endif

TOPDIR ?= $(CURDIR)
include $(DEVKITARM)/3ds_rules

#---------------------------------------------------------------------------------
# TARGET       output name
# BUILD        intermediate object dir
# SOURCES      dirs scanned for .c / .cpp / .s / .v.pica
# INCLUDES     dirs scanned for headers -- just `source`, since every #include
#              in this project is written relative to source/ (e.g.
#              "core/types.h", "vehicle/vehicle.h") rather than bare filenames,
#              so headers in different subsystem folders never collide.
# ROMFS        ships romfs/cacert.pem, the CA bundle source/net/http.c reads
#              once at init and hands to libcurl as an in-memory
#              CURLOPT_CAINFO_BLOB. It is a real shipped asset now, not the
#              empty placeholder it was in Phase 1 -- deleting it breaks
#              every HTTPS request the update check makes.
#---------------------------------------------------------------------------------
TARGET		:=	dirt2
BUILD		:=	build
SOURCES		:=	source source/core source/vehicle source/input source/render source/world source/race source/net source/update source/ui
DATA		:=	data
INCLUDES	:=	source
ROMFS		:=	romfs

APP_TITLE	:=	DiRT2 Clone
# This is the SMDH long description -- the line the Home Menu shows under the
# title, so it is user-facing on every install. It said "Phase 1 physics test"
# right up to v1.0.2, which was a build with a circuit, a lap clock and an
# update check in it. Keep it describing what the build actually IS.
APP_DESCRIPTION	:=	Rally driving on a closed circuit, against the clock
APP_AUTHOR	:=	steve

#---------------------------------------------------------------------------------
# options for code generation
#---------------------------------------------------------------------------------
# Old 3DS baseline (this project's stated Phase 1 target): armv6k/mpcore/hard-
# float/soft-thread-pointer is the standard devkitARM 3DS triple, same as both
# sibling projects. VFPv2 hardware float comes from -mfloat-abi=hard; there is
# no NEON on this core and nothing here should assume there is (see
# core/types.h's header comment on why f32 is a plain float, not fixed-point).
ARCH	:=	-march=armv6k -mtune=mpcore -mfloat-abi=hard -mtp=soft

# -Wall -Wextra -Werror, matching both sibling projects: a warning that scrolls
# past in the build log is a bug that ships on a console with no crash reporter.
# -O2 (devkitPro's own default) rather than mc's measured -O3 -- there is no
# Phase 1 measurement yet justifying trading code size for the ~2% mc measured,
# and this project has not shipped a single build to have a code-size budget to
# spend against. Revisit with a real measurement if it ever matters.
CFLAGS	:=	-g -Wall -Wextra -Werror -O2 -mword-relocations \
			-ffunction-sections \
			$(ARCH)

CFLAGS	+=	$(INCLUDE) -D__3DS__

# For one-off instrumented/debug builds without editing source, e.g.
#   make EXTRA_CFLAGS=-DPHYSICS_HZ=60
# See core/timestep.h for why PHYSICS_HZ is a compile-time override point.
CFLAGS	+=	$(EXTRA_CFLAGS)

CXXFLAGS	:= $(CFLAGS) -fno-rtti -fno-exceptions -std=gnu++11

ASFLAGS	:=	-g $(ARCH)
LDFLAGS	=	-specs=3dsx.specs -g $(ARCH) -Wl,-Map,$(notdir $*.map)

# LINK ORDER IS LOAD-BEARING. The devkitARM linker resolves static archives
# strictly left to right and does not re-scan an archive it has already passed,
# so every library must appear BEFORE the libraries it depends on.
#
# citro2d before citro3d: citro2d is built on top of citro3d, so citro2d's own
# citro3d calls need citro3d's symbols still to come. debugdraw.h's and hud.h's
# on-screen text is citro2d; everything else is bare citro3d.
#
# Then the networking stack for source/net/http.c, in dependency order:
#   curl        -> calls into mbedtls for TLS, zlib for Content-Encoding, and
#                  libctru for the actual BSD sockets (soc:U)
#   mbedtls     -> the TLS protocol layer, calls mbedx509
#   mbedx509    -> certificate parsing, calls mbedcrypto
#   mbedcrypto  -> the primitives; depends on nothing else here
#   z           -> zlib, pulled in by curl
#
# NOTE -lctru MOVED TO THE END, after the whole network stack. It used to sit
# third. citro2d/citro3d need it, but so does curl -- for socket/connect/recv
# via soc:U -- and libctru is the LAST consumer in the chain, so it has to be
# the last archive scanned. With -lctru back in its old position, curl's socket
# references resolve against an archive the linker has already walked past, and
# the link fails with undefined references to symbols that are visibly present
# in libctru.a. Loud rather than subtle, but easy to "fix" in the wrong place.
#
# WHY curl+mbedTLS AT ALL, rather than libctru's own httpc/ssl:C. This was
# MEASURED on steve's real console during the Skywave project, not assumed:
# the 3DS's ssl:C service tops out at TLS 1.1, and GitHub has required TLS 1.2+
# since 2018. github.com, api.github.com, objects.githubusercontent.com and
# release-assets.githubusercontent.com all returned a TLS "protocol version"
# alert. SSLCOPT_DisableVerify does NOT help -- it disables certificate
# checking, not protocol negotiation, so the handshake still never completes.
# curl over raw soc:U with its own bundled mbedTLS bypasses ssl:C entirely,
# which is the same route Universal-Updater and FBI take for the same reason.
LIBS	:= -lcitro2d -lcitro3d -lcurl -lmbedtls -lmbedx509 -lmbedcrypto -lz -lctru -lm

#---------------------------------------------------------------------------------
# list of directories containing libraries, this must be the top level containing
# include and lib
#---------------------------------------------------------------------------------
LIBDIRS	:= $(PORTLIBS) $(CTRULIB)

#---------------------------------------------------------------------------------
# CIA packaging (shape copied from model-making's Makefile).
#
# `make` produces only the .3dsx -- that's the fast day-to-day loop (Azahar,
# or a real console via 3dslink/homebrew launcher). `make cia` is a second
# pass through makerom/bannertool for an installable .cia. Both tools ship
# with devkitPro, called by full path since they are not on PATH inside a
# plain msys2 login shell.
#
# NOT YET BUILDABLE: this target needs three inputs nobody has supplied yet --
#   dirt2.rsf       the makerom spec (title, UniqueId, permissions)
#   cia/banner.png  256x128 Home Menu banner artwork
#   cia/banner.wav  the banner's jingle
# None of the three exist in this repo. Per this project's standing rule
# against fabricating supplied-asset placeholders, this task does not invent
# banner art, a jingle, or an .rsf's UniqueId -- those are asset/config
# decisions for steve to make, not to improvise. The target is wired up ready
# to work the moment those three files exist; until then `make cia` fails
# loudly on the missing prerequisites rather than silently producing nothing.
#---------------------------------------------------------------------------------
MAKEROM		:=	$(DEVKITPRO)/tools/bin/makerom
BANNERTOOL	:=	$(DEVKITPRO)/tools/bin/bannertool

#---------------------------------------------------------------------------------
# no real need to edit anything past this point unless you need to add
# additional rules for different file extensions
#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))
#---------------------------------------------------------------------------------

export OUTPUT	:=	$(CURDIR)/$(TARGET)
export TOPDIR	:=	$(CURDIR)

export VPATH	:=	$(foreach dir,$(SOURCES),$(CURDIR)/$(dir)) \
			$(foreach dir,$(DATA),$(CURDIR)/$(dir))

export DEPSDIR	:=	$(CURDIR)/$(BUILD)

CFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))
CPPFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.cpp)))
SFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.s)))
PICAFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.v.pica)))
SHLISTFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.shlist)))
BINFILES	:=	$(foreach dir,$(DATA),$(notdir $(wildcard $(dir)/*.*)))

#---------------------------------------------------------------------------------
# use CXX for linking C++ projects, CC for standard C
#---------------------------------------------------------------------------------
ifeq ($(strip $(CPPFILES)),)
#---------------------------------------------------------------------------------
	export LD	:=	$(CC)
#---------------------------------------------------------------------------------
else
#---------------------------------------------------------------------------------
	export LD	:=	$(CXX)
#---------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------

export OFILES_SOURCES 	:=	$(CPPFILES:.cpp=.o) $(CFILES:.c=.o) $(SFILES:.s=.o)

export OFILES_BIN	:=	$(addsuffix .o,$(BINFILES)) \
			$(PICAFILES:.v.pica=.shbin.o) $(SHLISTFILES:.shlist=.shbin.o)

export OFILES := $(OFILES_BIN) $(OFILES_SOURCES)

export HFILES	:=	$(PICAFILES:.v.pica=_shbin.h) $(SHLISTFILES:.shlist=_shbin.h) \
			$(addsuffix .h,$(subst .,_,$(BINFILES)))

export INCLUDE	:=	$(foreach dir,$(INCLUDES),-I$(CURDIR)/$(dir)) \
			$(foreach dir,$(LIBDIRS),-I$(dir)/include) \
			-I$(CURDIR)/$(BUILD)

export LIBPATHS	:=	$(foreach dir,$(LIBDIRS),-L$(dir)/lib)

export _3DSXDEPS	:=	$(if $(NO_SMDH),,$(OUTPUT).smdh)

# No ICON supplied (see the cia: target's note above -- no asset fabricated
# on spec) -- devkitARM's smdhtool falls back to libctru's own default icon
# when APP_ICON is unset, so the .3dsx still builds and is installable, just
# with a placeholder icon until real art exists.
ifeq ($(strip $(ICON)),)
	icons := $(wildcard *.png)
	ifneq (,$(findstring $(TARGET).png,$(icons)))
		export APP_ICON := $(TOPDIR)/$(TARGET).png
	else
		ifneq (,$(findstring icon.png,$(icons)))
			export APP_ICON := $(TOPDIR)/icon.png
		endif
	endif
else
	export APP_ICON := $(TOPDIR)/$(ICON)
endif

ifeq ($(strip $(NO_SMDH)),)
	export _3DSXFLAGS += --smdh=$(CURDIR)/$(TARGET).smdh
endif

ifneq ($(ROMFS),)
	export _3DSXFLAGS += --romfs=$(CURDIR)/$(ROMFS)
endif

.PHONY: all clean cia

#---------------------------------------------------------------------------------
all: $(BUILD)
	@$(MAKE) --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

#---------------------------------------------------------------------------------
cia: all $(OUTPUT).cia

$(OUTPUT).banner	:	cia/banner.png cia/banner.wav
	@echo banner ...
	@$(BANNERTOOL) makebanner -i cia/banner.png -a cia/banner.wav -o $@

$(OUTPUT).cia	:	$(OUTPUT).elf $(OUTPUT).smdh $(OUTPUT).banner dirt2.rsf
	@echo $(notdir $@) ...
	@$(MAKEROM) -f cia -o $@ -elf $(OUTPUT).elf -rsf dirt2.rsf \
		-DAPP_ROMFS=$(ROMFS) \
		-icon $(OUTPUT).smdh -banner $(OUTPUT).banner -exefslogo -target t
	@echo built ... $(notdir $@)

$(BUILD):
	@mkdir -p $@

#---------------------------------------------------------------------------------
clean:
	@echo clean ...
	@rm -fr $(BUILD) $(TARGET).3dsx $(OUTPUT).smdh $(TARGET).elf \
		$(OUTPUT).cia $(OUTPUT).banner

#---------------------------------------------------------------------------------
else

#---------------------------------------------------------------------------------
# main targets
#---------------------------------------------------------------------------------
$(OUTPUT).3dsx	:	$(OUTPUT).elf $(_3DSXDEPS)

$(OFILES_SOURCES) : $(HFILES)

$(OUTPUT).elf	:	$(OFILES)

#---------------------------------------------------------------------------------
# you need a rule like this for each extension you use as binary data
#---------------------------------------------------------------------------------
%.bin.o	%_bin.h :	%.bin
#---------------------------------------------------------------------------------
	@echo $(notdir $<)
	@$(bin2o)

#---------------------------------------------------------------------------------
.PRECIOUS	:	%.shbin
#---------------------------------------------------------------------------------
%.shbin.o %_shbin.h : %.shbin
#---------------------------------------------------------------------------------
	$(SILENTMSG) $(notdir $<)
	$(bin2o)

-include $(DEPSDIR)/*.d

#---------------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------------
