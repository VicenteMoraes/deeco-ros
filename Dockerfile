FROM osrf/ros2:nightly
RUN apt update && apt full-upgrade -y

# Installing Requirements
RUN apt install -y curl gnupg2 lsb-release
RUN wget https://raw.githubusercontent.com/VicenteMoraes/deeco-ros/master/requirements.txt
RUN pip3 install -r /requirements.txt

# Making ros package
RUN mkdir -p action_ws/src
RUN bash -c "source /opt/ros/rolling/setup.bash && cd action_ws/src && ros2 pkg create deeco_actions"
COPY mini_deeco/action_ws/action action_ws/src/deeco_actions/action
COPY mini_deeco/action_ws/CMakeLists.txt action_ws/src/deeco_actions/CMakeLists.txt
COPY mini_deeco/action_ws/package.xml action_ws/src/deeco_actions/package.xml
RUN bash -c "source /opt/ros/rolling/setup.bash && cd action_ws && colcon build"
COPY mini_deeco/action_ws/ros_entrypoint.sh /new_entrypoint.sh
RUN bash -c 'cat new_entrypoint.sh > ros_entrypoint.sh'