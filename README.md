# aart-os
mcu OS written in rust

See DESIGN.md for the architecture and milestone plan.

Hardware-free logic (scheduler, commutation state machine, etc.) lives in
`aart-core` and is tested natively on the host, no emulator required:  
cd aart-core  
cargo test

Firmware (`stm32_os`) tests:  
cd stm32_os  
cargo clean   
cargo test --target thumbv7em-none-eabihf --features qemu   

Docker build (run from the repo root — stm32_os depends on the sibling
aart-core crate, so the build context can't be stm32_os/ alone):  
docker build -t stm32-os -f stm32_os/Dockerfile .   
docker run --rm stm32-os   
