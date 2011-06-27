#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_vector_fltk)/lib
	cp $(shell rospack find cisst)/lib/libcisstVectorFLTK.a $(shell rospack find cisst_vector_fltk)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
