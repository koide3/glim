FROM koide3/gtsam_docker:focal_cuda11.2

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    libspdlog-dev libopencv-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.lld 50

# without viewer and CUDA
COPY . /root/glim
WORKDIR /root/glim/build
RUN CC=clang CXX=clang++ cmake .. \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_VIEWER=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)

# with viewer
WORKDIR /root/glim/build
RUN CC=clang CXX=clang++ cmake .. \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)

# with CUDA
WORKDIR /root/glim/build
RUN CC=clang CXX=clang++ cmake .. \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)

CMD ["bash"]
