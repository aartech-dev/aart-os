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

docker build -t stm32-os .   
docker run --rm stm32-os   
