FROM koide3/gtsam_docker:jammy_cuda11.8

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
