# Verilator 5 + clang/lld for sim/ (docker build -t mister-p8-verilator sim)
FROM ubuntu:24.04
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends verilator g++ make zlib1g-dev ca-certificates clang lld && rm -rf /var/lib/apt/lists/*
