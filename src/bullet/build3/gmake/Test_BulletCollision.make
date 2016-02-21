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
  OBJDIR     = obj/x64/Release/Test_BulletCollision
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_BulletCollision_gmake_x64_release
  DEFINES   += 
  INCLUDES  += -I../../test/collision -I../../src -I../../test/gtest-1.7.0/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -msse2 -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -s -m64 -L/usr/lib64 -L../../bin
  LIBS      += ../../bin/libLinearMath_gmake_x64_release.a ../../bin/libgtest_gmake_x64_release.a -lpthread
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LDDEPS    += ../../bin/libLinearMath_gmake_x64_release.a ../../bin/libgtest_gmake_x64_release.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS) $(RESOURCES) $(ARCH) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debug64)
  OBJDIR     = obj/x64/Debug/Test_BulletCollision
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_BulletCollision_gmake_x64_debug
  DEFINES   += -D_DEBUG=1
  INCLUDES  += -I../../test/collision -I../../src -I../../test/gtest-1.7.0/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -m64 -L/usr/lib64 -L../../bin
  LIBS      += ../../bin/libLinearMath_gmake_x64_debug.a ../../bin/libgtest_gmake_x64_debug.a -lpthread
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LDDEPS    += ../../bin/libLinearMath_gmake_x64_debug.a ../../bin/libgtest_gmake_x64_debug.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS) $(RESOURCES) $(ARCH) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/main.o \
	$(OBJDIR)/btVoronoiSimplexSolver.o \
	$(OBJDIR)/btSphereShape.o \
	$(OBJDIR)/btMultiSphereShape.o \
	$(OBJDIR)/btPolyhedralConvexShape.o \
	$(OBJDIR)/btConvexShape.o \
	$(OBJDIR)/btConvexInternalShape.o \
	$(OBJDIR)/btCollisionShape.o \
	$(OBJDIR)/btConvexPolyhedron.o \

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
	@echo Linking Test_BulletCollision
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
	@echo Cleaning Test_BulletCollision
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

$(OBJDIR)/main.o: ../../test/collision/main.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btVoronoiSimplexSolver.o: ../../src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btSphereShape.o: ../../src/BulletCollision/CollisionShapes/btSphereShape.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btMultiSphereShape.o: ../../src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btPolyhedralConvexShape.o: ../../src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btConvexShape.o: ../../src/BulletCollision/CollisionShapes/btConvexShape.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btConvexInternalShape.o: ../../src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btCollisionShape.o: ../../src/BulletCollision/CollisionShapes/btCollisionShape.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/btConvexPolyhedron.o: ../../src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"

-include $(OBJECTS:%.o=%.d)
