# https://medium.com/@aharon.amir/develop-c-on-docker-with-vscode-98fb85b818b2
FROM ubuntu:latest

RUN apt-get update && \
    apt-get install -y git wget gcc-8 g++-8 cmake gdb gdbserver  && \
    apt-get clean autoclean && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*
    
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 999 \
 && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 999 \
 && update-alternatives --install /usr/bin/cc  cc  /usr/bin/gcc-8 999 \
 && update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-8 999

RUN useradd -ms /bin/bash develop
RUN echo "develop   ALL=(ALL:ALL) ALL" >> /etc/sudoers

# for gdbserver
EXPOSE 2000

USER develop
WORKDIR /home/develop
RUN git clone https://github.com/google/googletest.git

RUN wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-a/8.3-2019.03/binrel/gcc-arm-8.3-2019.03-x86_64-arm-eabi.tar.xz
RUN tar xf gcc-arm-8.3-2019.03-x86_64-arm-eabi.tar.xz && \
    rm gcc-arm-8.3-2019.03-x86_64-arm-eabi.tar.xz
ENV PATH="/home/develop/gcc-arm-8.3-2019.03-x86_64-arm-eabi/bin:${PATH}"

ARG WORKSPACE_ROOT
VOLUME ${WORKSPACE_ROOT}
WORKDIR ${WORKSPACE_ROOT}/build