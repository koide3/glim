FROM nvidia/cuda:11.1-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

RUN apt-get update \
  && apt-get upgrade -y \
  # add gtsam respository
  && apt-get install --no-install-recommends -y \
    software-properties-common \
  && add-apt-repository ppa:borglab/gtsam-develop \
  # install dependencies
  && apt-get install --no-install-recommends -y \
    wget nano build-essential \
    git cmake libeigen3-dev ca-certificates \
    libtbb-dev libboost-all-dev libgtest-dev libmetis-dev \
    libgtsam-dev libgtsam-unstable-dev \
    libglm-dev libglfw3-dev libpng-dev libopencv-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*


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
