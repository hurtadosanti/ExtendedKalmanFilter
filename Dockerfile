FROM ubuntu:20.04
ENV TZ=Europe/Berlin
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y openssh-server \
 build-essential gcc g++ \
 gdb clang cmake rsync \
 tar wget \
 && apt-get clean
RUN mkdir /var/run/sshd
RUN echo 'root:wLrebr2*riWESW' | chpasswd
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed -i 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' /etc/pam.d/sshd

ENV NOTVISIBLE="in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

WORKDIR /development

RUN wget -nv https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.gz \
 && tar xvfz eigen-3.3.8.tar.gz && cd eigen-3.3.8 && mkdir build && cd build && cmake .. && make install

EXPOSE 22
CMD ["/usr/sbin/sshd", "-D"]