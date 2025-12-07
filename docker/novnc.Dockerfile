FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    xfce4 xfce4-goodies x11vnc novnc websockify \
    supervisor net-tools \
    gazebo \
    && rm -rf /var/lib/apt/lists/*

# Set up noVNC
RUN ln -s /usr/share/novnc/vnc.html /usr/share/novnc/index.html

COPY compose/novnc/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

ENV USER=root
ENV DISPLAY=:1
ENV VNC_PORT=5900
ENV NOVNC_PORT=8080
ENV GAZEBO_MASTER_URI=http://sim:11345
ENV ROS_DOMAIN_ID=23

EXPOSE 8080

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
