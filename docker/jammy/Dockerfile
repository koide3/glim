FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

RUN apt-get update \
  && apt-get upgrade -y \
  # install dependencies
  && apt-get install --no-install-recommends -y \
  wget nano build-essential \
  git cmake libeigen3-dev ca-certificates \
  libtbb-dev libboost-all-dev libgtest-dev libmetis-dev \
  libglm-dev libglfw3-dev libpng-dev libopencv-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# install GTSAM
WORKDIR /root
RUN git clone https://github.com/borglab/gtsam.git
WORKDIR /root/gtsam/build
RUN cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
RUN make -j$(nproc) && make install


# without viewer and CUDA
COPY . /root/glim
WORKDIR /root/glim/build
RUN cmake .. \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_VIEWER=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)


# with viewer
WORKDIR /root
RUN git clone https://github.com/koide3/iridescence.git --recursive
WORKDIR /root/iridescence/build
RUN cmake ..
RUN make -j$(nproc) && make install

WORKDIR /root/glim/build
RUN cmake .. \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)


# with CUDA
WORKDIR /root/glim/build
RUN cmake .. \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)

CMD ["bash"]
