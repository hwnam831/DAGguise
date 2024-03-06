FROM ubuntu:18.04 as gem5-env
LABEL maintainer="Peter Deutsch (@peterdeutsch)"
LABEL description="a unified environment for compiling & running gem5"

RUN apt update && \
    apt install -y --no-install-recommends \
    build-essential \
    git \
    m4 \
    zlib1g \
    zlib1g-dev \
    libjpeg9-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libprotoc-dev \
    libgoogle-perftools-dev \
    python-dev \
    python-six \
    python3-six \
    python \
    python-pip \
    libboost-all-dev \
    pkg-config \
    wget \
    curl \
    vim \
    gosu \
    sudo \
    gcc \
    gdb \
    libpng-dev \
    cmake \
    python3 \
    python3-venv

RUN pip install --upgrade pip && pip install scons && \
    cp -r /usr/local/lib/python2.7/dist-packages/scons* \
          /usr/local/lib/python2.7/site-packages/

FROM gem5-env as dagguise-env
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]

FROM gem5-env as dagguise
RUN adduser --uid 9001 --disabled-password user && \
    cp /etc/sudoers /etc/sudoers.bak && \
    echo 'user  ALL=(root) NOPASSWD: ALL' >> /etc/sudoers
USER user
WORKDIR /home/user

ENV GEM5_ROOT=/home/user/dagguise
ENV WORKLOADS_ROOT=/home/user/SPEC_2017

WORKDIR /home/user

RUN mkdir .virtualenv && cd .virtualenv && \
    python3 -m venv asplos && \
    echo "source /home/user/.virtualenv/asplos/bin/activate" >> /home/user/.bashrc

COPY --chown=user . /home/user/dagguise
WORKDIR /home/user/dagguise
RUN . /home/user/.virtualenv/asplos/bin/activate && \
    pip3 install -r eval_scripts/requirements.txt 
    
RUN scons build/X86/gem5.opt -j$(nproc)
CMD ["/bin/bash"]

