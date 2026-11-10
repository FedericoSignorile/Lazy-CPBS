FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates \
      libyaml-cpp-dev libcgal-dev libgmp-dev libmpfr-dev libboost-dev \
      python3 python3-matplotlib python3-numpy ffmpeg \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /lazy-cpbs
COPY . .
RUN rm -rf build && cmake -S . -B build && cmake --build build -j"$(nproc)"
