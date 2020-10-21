FROM ubuntu:20.04
ENV TZ=Europe/Berlin
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -qq update && apt-get -yqq upgrade && apt-get install -yqq git libuv1-dev libssl-dev zlib1g-dev gcc g++ cmake make
WORKDIR /opt
RUN git clone https://github.com/uWebSockets/uWebSockets && cd uWebSockets && git checkout e94b6e1 \
    && mkdir build && cd build && cmake .. && make && make install \
    && cd ../.. && ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so && rm -r uWebSockets