all: lbfgs

TARBALL = build/liblbfgs-1.10.tar.gz
TARBALL_URL = https://github.com/downloads/chokkan/liblbfgs/liblbfgs-1.10.tar.gz
SOURCE_DIR = build/liblbfgs-1.10
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

lbfgs: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=$(PWD) && make && make install
	mkdir -p include/lbfgs
	mv include/lbfgs.h include/lbfgs
	touch lbfgs
clean:
	-rm -rf $(SOURCE_DIR) lbfgs include lib share
wipe: clean
	-rm -rf build
