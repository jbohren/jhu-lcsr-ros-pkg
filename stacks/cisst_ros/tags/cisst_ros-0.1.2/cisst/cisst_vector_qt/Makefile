#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_vector_qt)/lib
	cp $(shell rospack find cisst)/lib/libcisstVectorQt.a $(shell rospack find cisst_vector_qt)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
