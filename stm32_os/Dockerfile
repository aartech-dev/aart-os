FROM rust:1.75

RUN apt-get update && apt-get install -y \
    qemu-system-arm \
    gcc-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

RUN rustup target add thumbv7em-none-eabihf

WORKDIR /app

COPY Cargo.toml Cargo.lock ./
COPY .cargo .cargo
RUN mkdir -p src && echo "fn main() {}" > src/main.rs
RUN cargo build --target thumbv7em-none-eabihf || true
RUN rm -rf src

COPY src src
COPY tests tests
COPY memory.x memory.x

CMD ["cargo", "test", "--target", "thumbv7em-none-eabihf", "--", "--nocapture"]
