#!/bin/bash

docker run -it --rm --net=host \
	   --privileged \
	   --name duckiefloat-rpi \
           -v ${HOME}/subt-duckiefloat:/root/subt-duckiefloat \
	   -v /dev:/dev \
	   -w /root/subt-duckiefloat \
           argnctu/subt:arm32-duckiefloat
