all: lwpr_lib

PKGVERSION = lwpr-1.2.3
TARBALL = build/$(PKGVERSION).tar.gz
TARBALL_URL = http://www.ipab.inf.ed.ac.uk/slmc/software/lwpr/$(PKGVERSION).tar.gz
SOURCE_DIR = build/$(PKGVERSION)
MD5SUM_FILE = $(PKGVERSION).tar.gz.md5sum
UNPACK_CMD = tar xzf

PATCH = lwpr-1.2.3.patch

CONFIGURE = ./configure --prefix=$(shell rospack find lwpr_lib) --with-matlab=/opt/matlab --enable-threads=8
# CONFIGURE = ./configure --prefix=$(shell rospack find lwpr_lib)

$(VERSION)/Makefile: $(VERSION)/Makefile.in $(VERSION)/configure $(VERSION)/configure.in
	cd $(VERSION) && $(CONFIGURE)

lwpr_lib:
	cd `rospack find lwpr_lib` && rm -rf include && rm -rf lib && cd build && rm -rf $(PKGVERSION) && $(UNPACK_CMD) $(PKGVERSION).tar.gz && cd $(PKGVERSION) && patch -p1 -i ../../$(PATCH) && $(CONFIGURE) && make $(ROS_PARALLEL_JOBS) && make install && cd `rospack find lwpr_lib` && cp -r include/ lwpr_lib && mv lwpr_lib/ include/ && touch lwpr_lib

clean:
	rm -rf build/$(PKGVERSION) lib include
	rm -f lwpr_lib

wipe: clean
