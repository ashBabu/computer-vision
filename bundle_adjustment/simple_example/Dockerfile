FROM debian:bookworm-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libceres-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up project
WORKDIR /bundle_adjustment
COPY . .

# Compile bundle adjustment program
RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Run the program
# ENTRYPOINT ["./bundle_adjustment"]
CMD ["./build/bundle_adjustment"]