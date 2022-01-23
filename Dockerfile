FROM osrf/ros2:nightly
RUN apt update && apt full-upgrade -y
RUN apt install -y curl gnupg2 lsb-release
RUN wget https://raw.githubusercontent.com/VicenteMoraes/deeco-ros/master/requirements.txt
RUN pip3 install -r /requirements.txt