# GNU Make project makefile autogenerated by Premake
ifndef config
  config=release64
endif

ifndef verbose
  SILENT = @
endif

ifndef CC
  CC = gcc
endif

ifndef CXX
  CXX = g++
endif

ifndef AR
  AR = ar
endif

ifeq ($(config),release64)
  OBJDIR     = obj/x64/Release/Test_SharedMemoryPhysicsClient
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_SharedMemoryPhysicsClient_gmake_x64_release
  DEFINES   += 
  INCLUDES  += -I../../src -I../../examples/SharedMemory
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -msse2 -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -s -m64 -L/usr/lib64 -L../../bin
  LIBS      += ../../bin/libBulletFileLoader_gmake_x64_release.a ../../bin/libBullet3Common_gmake_x64_release.a ../../bin/libLinearMath_gmake_x64_release.a
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LDDEPS    += ../../bin/libBulletFileLoader_gmake_x64_release.a ../../bin/libBullet3Common_gmake_x64_release.a ../../bin/libLinearMath_gmake_x64_release.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS) $(RESOURCES) $(ARCH) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debug64)
  OBJDIR     = obj/x64/Debug/Test_SharedMemoryPhysicsClient
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_SharedMemoryPhysicsClient_gmake_x64_debug
  DEFINES   += -D_DEBUG=1
  INCLUDES  += -I../../src -I../../examples/SharedMemory
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -m64 -L/usr/lib64 -L../../bin
  LIBS      += ../../bin/libBulletFileLoader_gmake_x64_debug.a ../../bin/libBullet3Common_gmake_x64_debug.a ../../bin/libLinearMath_gmake_x64_debug.a
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LDDEPS    += ../../bin/libBulletFileLoader_gmake_x64_debug.a ../../bin/libBullet3Common_gmake_x64_debug.a ../../bin/libLinearMath_gmake_x64_debug.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS) $(RESOURCES) $(ARCH) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/test.o \
	$(OBJDIR)/PhysicsClient.o \
	$(OBJDIR)/PhysicsClientC_API.o \
	$(OBJDIR)/Win32SharedMemory.o \
	$(OBJDIR)/PosixSharedMemory.o \
	$(OBJDIR)/b3ResourcePath.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking Test_SharedMemoryPhysicsClient
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning Test_SharedMemoryPhysicsClient
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
	-$(SILENT) cp $< $(OBJDIR)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
endif

$(OBJDIR)/test.o: ../../test/SharedMemory/test.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -c "$<"
$(OBJDIR)/PhysicsClient.o: ../../examples/SharedMemory/PhysicsClient.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/PhysicsClientC_API.o: ../../examples/SharedMemory/PhysicsClientC_API.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/Win32SharedMemory.o: ../../examples/SharedMemory/Win32SharedMemory.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/PosixSharedMemory.o: ../../examples/SharedMemory/PosixSharedMemory.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/b3ResourcePath.o: ../../examples/Utils/b3ResourcePath.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"

-include $(OBJECTS:%.o=%.d)
