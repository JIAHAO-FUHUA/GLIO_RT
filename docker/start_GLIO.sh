#!/bin/bash

docker container ls -a -f name=glio_ublox | grep glio_ublox$ > /dev/null

if [ $? == 0 ]
then
	docker container start glio_ublox
	docker exec -it glio_ublox /bin/bash

else
	xhost +
	#docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE -v /home/aae/GLIO_RT_ws:/root/GLIO_RT_ws --device /dev/ttyACM0 --name glio_ublox docker_glio /bin/bash
	docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE -v /home/aae/GLIO_RT_ws:/root/GLIO_RT_ws --name glio_ublox docker_glio /bin/bash
fi
