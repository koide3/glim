FROM koide3/gtsam_docker:jammy_cuda11.8

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

# resolve omp_is_initial_device-related errors
RUN sed -i '496{s/^/\/\//}' /usr/lib/llvm-14/lib/clang/14.0.0/include/omp.h

# with CUDA
WORKDIR /root/glim/build
RUN CC=clang CXX=clang++ cmake .. \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)

CMD ["bash"]
