# aart-os
mcu OS written in rust
cd stm32_os  
cargo clean   
cargo test --target thumbv7em-none-eabihf --features qemu   

docker build -t stm32-os .   
docker run --rm stm32-os   
