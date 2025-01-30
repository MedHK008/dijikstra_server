FROM gcc:latest

WORKDIR /app

# Install build tools
RUN apt-get update && apt-get install -y cmake ninja-build

# Copy source and libraries
COPY . .

# Build
RUN mkdir build && \
    cd build && \
    cmake -G Ninja .. && \
    cmake --build .

EXPOSE 8080

CMD ["./build/Dijikstra_backend"]