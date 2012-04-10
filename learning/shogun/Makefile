all:
	@if [ -e "shogun_installed" ]; then echo "Shogun already build, skipping"; else \
  cd `rospack find shogun` && \
  rm -rf include && rm -rf lib && cd shogun/src && \
  ./configure --disable-doxygen --prefix=`rospack find shogun` --interfaces=cmdline_static --enable-hmm-parallel --enable-debug --enable-lapack && \
  make $(ROS_PARALLEL_JOBS) && make install && \
  touch ../../shogun_installed; \
  fi

clean:
	rm -rf lib include
	cd shogun/src && make clean
	rm -f shogun_installed
	rm -f shogun/src/shogun/lib/versionstring.h

wipe: clean
