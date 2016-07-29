#!/bin/bash

 mkdir bullet-build
 cd bullet-build
 cmake .. -G "Unix Makefiles" -DINSTALL_LIBS=ON 
 make -j4
 sudo make install
